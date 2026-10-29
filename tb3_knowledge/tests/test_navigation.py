"""Controller safety and simulator/learning contracts, without ROS or maps."""
import math
from dataclasses import replace
from unittest import TestCase
from unittest.mock import patch

from tb3_knowledge.core.model import Arena, Hardware, Policy, Pose
from tb3_knowledge.core.navigation import Navigator, Observation
from tb3_knowledge.core.clearance import UnknownSector
from tb3_knowledge.core.simulation import run_episode
from tb3_knowledge.core.training import train


class AnalyticWorld:
    """Independent rectangular room and circular obstacle raycaster."""
    def __init__(self, size=2.0, obstacle=None):
        self.size, self.obstacle = size, obstacle

    def free(self, x, y, radius):
        if not radius <= x <= self.size-radius or not radius <= y <= self.size-radius:
            return False
        if self.obstacle:
            ox, oy, r = self.obstacle
            return math.hypot(x-ox, y-oy) > r+radius
        return True

    def scan(self, pose, count, max_range):
        ranges = []
        for index in range(count):
            angle = pose.yaw - math.pi + index*2*math.pi/count
            dx, dy = math.cos(angle), math.sin(angle)
            intersections = [max_range]
            if dx > 1e-9:
                intersections.append((self.size-pose.x)/dx)
            elif dx < -1e-9:
                intersections.append(-pose.x/dx)
            if dy > 1e-9:
                intersections.append((self.size-pose.y)/dy)
            elif dy < -1e-9:
                intersections.append(-pose.y/dy)
            if self.obstacle:
                ox, oy, radius = self.obstacle
                projection = (ox-pose.x)*dx+(oy-pose.y)*dy
                perpendicular2 = (ox-pose.x)**2+(oy-pose.y)**2-projection**2
                if perpendicular2 < radius**2:
                    hit = projection-math.sqrt(radius**2-perpendicular2)
                    if hit > 0:
                        intersections.append(hit)
            ranges.append(min(intersections))
        return tuple(ranges)

    def reachable(self, start, goal, radius):
        return self.free(start.x, start.y, radius) and self.free(*goal, radius)


class NavigationTests(TestCase):
    def setUp(self):
        self.arena, self.hardware = Arena(2, 2), Hardware()

    def navigator(self, pose=None, goal=(1.7, 1.0)):
        pose = pose or Pose(0.5, 1, 0)
        return Navigator(self.arena, goal, Policy(), self.hardware), Observation(pose, (3.0,)*120)

    def test_stale_future_and_invalid_scan_stop(self):
        nav, observation = self.navigator()
        self.assertEqual(nav.command(observation, 0.7), (0, 0))
        self.assertEqual(nav.reason, 'stale_scan')
        self.assertEqual(nav.command(observation, -0.1), (0, 0))
        self.assertEqual(nav.reason, 'future_scan')
        observation = Observation(Pose(.5, 1), (float('nan'),)*120)
        self.assertEqual(nav.command(observation, 0), (0, 0))
        self.assertEqual(nav.reason, 'invalid_scan')

    def test_partial_scan_does_not_invent_missing_coverage(self):
        nav, _ = self.navigator()
        observation = Observation(Pose(.5, 1), (3.0,)*60, angle_min=-math.pi/2, angle_increment=math.pi/60)
        self.assertEqual(nav.command(observation, 0), (0, 0))
        self.assertEqual(nav.reason, 'incomplete_scan')

    def test_scan_coverage_accepts_exactly_90_percent_and_waits_below(self):
        for missing,allowed in ((12,True),(13,False)):
            nav,obs=self.navigator()
            obs=replace(obs,ranges=(float('nan'),)*missing+(3.,)*(120-missing),mount=(-.032,0.,0.))
            velocity,angular=nav.command(obs,0.)
            if allowed:
                self.assertGreater(velocity,0.)
            else:
                self.assertEqual((velocity,angular),(0.,0.))
                self.assertEqual(nav.reason,'insufficient_scan_coverage')

    def test_scan_coverage_uses_reported_raw_score_and_rejects_invalid_scores(self):
        for score,reason in ((89.9,'insufficient_scan_coverage'),(float('nan'),'invalid_observation'),
                (-1.,'invalid_observation'),(101.,'invalid_observation')):
            nav,obs=self.navigator()
            self.assertEqual(nav.command(replace(obs,scan_coverage_percent=score),0.),(0.,0.))
            self.assertEqual(nav.reason,reason)

    def test_history_spacing_can_only_increase_obstacle_inflation(self):
        nav,observation=self.navigator()
        nav.command(replace(observation,beam_spacing=.01),0.)
        self.assertAlmostEqual(nav._beam_width,2*math.pi/120)
        nav.command(replace(observation,beam_spacing=.12),0.)
        self.assertEqual(nav._beam_width,.12)
        self.assertEqual(nav.command(replace(observation,beam_spacing=float('nan')),0.),(0,0))
        self.assertEqual(nav.reason,'invalid_scan_angles')
        partial=replace(observation,ranges=(3.,)*60,angle_increment=math.pi/60,beam_spacing=.12)
        self.assertEqual(nav.command(partial,0.),(0,0))
        self.assertEqual(nav.reason,'incomplete_scan')

    def test_body_clearance_blocks_rotation(self):
        nav, _ = self.navigator(goal=(.5, 1.7))
        ranges = [3.0]*120
        ranges[0] = 0.10
        observation = Observation(Pose(.5, 1), tuple(ranges))
        self.assertEqual(nav.command(observation, 0), (0, 0))
        self.assertEqual(nav.reason, 'footprint_clearance')

    def test_passing_coverage_uses_measured_returns_despite_missing_ray(self):
        nav, _ = self.navigator(goal=(.5, 1.7))
        ranges = [3.0]*120
        ranges[0] = float('inf')
        observation = Observation(Pose(.5, 1), tuple(ranges), mount=(.05, 0, 0))
        self.assertNotEqual(nav.command(observation, 0), (0, 0))
        self.assertEqual(nav.reason, 'navigating')

    def test_single_clear_ray_is_not_a_passage(self):
        nav, _ = self.navigator()
        # Two obstacle edges leave a 0.20 m opening, smaller than safe diameter.
        points = ((.35, -.10), (.35, .10))
        observation = Observation(Pose(.5, 1), (3.0,)*120, points=points)
        clearance = nav._clearance(0, observation.pose, points, (), .6)
        self.assertLess(clearance, .35)
        self.assertFalse(clearance >= .6)

    def test_curved_motion_cannot_enter_unknown_sector(self):
        nav,obs=self.navigator()
        sector=UnknownSector((0.,0.),.5,.01,.3)
        self.assertFalse(nav._arc_clear(obs.pose,(),.3,1.,2.,(sector,)))

    def test_inspection_turn_keeps_measured_obstacle_padding_during_residual_drift(self):
        nav,obs=self.navigator()
        radius=self.hardware.safe_radius+.012
        points=tuple((radius*math.cos(i*math.pi/30),radius*math.sin(i*math.pi/30)) for i in range(60))
        obs=replace(obs,points=points,unknown_sectors=(),linear_speed=.02)
        self.assertEqual(nav.command(obs,0.),(0.,0.))
        self.assertEqual(nav.reason,'rotation_drift_clearance')

    def scan_advance_scene(self):
        nav,obs=self.navigator()
        points=tuple((.27*math.cos(a),.27*math.sin(a)) for a in (i*math.pi/30 for i in range(60)))
        sector=UnknownSector((-.032,0.),math.pi,.02,.16)
        return nav,replace(obs,points=points,unknown_sectors=(sector,))

    def test_complete_evidence_allows_short_passage_with_braking_checks(self):
        nav,obs=self.scan_advance_scene()
        obs=replace(obs,unknown_sectors=())
        velocity,angular=nav.command(obs,0.)
        self.assertGreater(velocity,0.)
        self.assertGreater(nav.candidate_count,0)
        self.assertEqual(nav.reason,'navigating')
        clearance=nav._clearance(0.,obs.pose,obs.points,(),.13)
        self.assertGreaterEqual(clearance,.05)
        self.assertLess(clearance,.13)
        latency=nav.hardware.command_timeout+nav.hardware.control_period
        self.assertLessEqual(velocity*latency+velocity**2/(2*nav.hardware.braking),clearance-.025)
        self.assertTrue(nav._arc_clear(obs.pose,obs.points,velocity,angular,latency+velocity/nav.hardware.braking,()))

    def test_sparse_scan_does_not_open_sub_body_width_gap(self):
        # 60 rays alias a 0.22 m wall opening. Rays at +/-12 degrees pass,
        # while +/-18-degree returns land at y=+/-0.162 m. Raw endpoint
        # capsules falsely admit the 0.226 m diameter robot through this gap.
        hardware = replace(self.hardware, scan_beams=60, safety_margin=.015)
        nav = Navigator(self.arena, (1.7, 1), Policy(), hardware)
        ranges = []
        for index in range(60):
            angle = -math.pi + index * 2*math.pi/60
            hit = .5/math.cos(angle) if math.cos(angle) > 1e-8 else 3.0
            y = hit*math.sin(angle)
            ranges.append(3.0 if abs(y) < .11 else min(3.0, hit))
        observation = Observation(Pose(.5, 1), tuple(ranges))
        points, invalid = observation.sensor_points(hardware.scan_range)
        self.assertLess(nav._clearance(0, observation.pose, points, invalid, .6), .5)
        nav.command(observation, 0)
        if nav.local_path:
            end_x, end_y = nav.local_path[-1]
            self.assertFalse(end_x > 1 and abs(end_y-1) < hardware.radius)

    def test_failed_progress_tries_bounded_recovery_then_stops(self):
        nav = Navigator(self.arena, (1.7, 1), replace(Policy(), progress_window=3), self.hardware)
        for index in range(340):
            now = index*.05
            nav.command(Observation(Pose(.5, 1), (3.0,)*120, stamp=now), now)
        self.assertEqual(nav.reason, 'recovery_exhausted')
        self.assertEqual(nav.mode, 'STOPPED')
        self.assertEqual(nav.recoveries, 5)

    def test_bounds_and_arrival_are_terminal(self):
        nav, _ = self.navigator()
        self.assertEqual(nav.command(Observation(Pose(.1, 1), (3.0,)*120), 0), (0, 0))
        self.assertEqual(nav.reason, 'outside_safe_arena')
        nav, obs = self.navigator(goal=(.52, 1))
        self.assertEqual(nav.command(obs, 0), (0, 0))
        self.assertEqual(nav.mode, 'ARRIVED')

    def test_braking_limit_and_finite_commands(self):
        nav, _ = self.navigator()
        points = ((.24, 0),)
        observation = Observation(Pose(.5, 1), (3.0,)*120, points=points, linear_speed=.08)
        for index in range(8):
            now = index*.05
            observation = Observation(observation.pose, observation.ranges, stamp=now,
                points=points, linear_speed=.08)
            velocity, angular = nav.command(observation, now)
            self.assertTrue(math.isfinite(velocity) and math.isfinite(angular))
            self.assertLessEqual(velocity, self.hardware.max_speed)
            self.assertLessEqual(abs(angular), self.hardware.max_angular_speed)
            if abs(angular) < .01:
                stopping = velocity*.4+velocity**2/(2*self.hardware.braking)
                self.assertLessEqual(stopping, .24-self.hardware.safe_radius+.005)

    def test_pause_restarts_acceleration_ramp_after_external_stop(self):
        nav, observation = self.navigator()
        for index in range(20):
            now = index * self.hardware.control_period
            nav.command(Observation(observation.pose, observation.ranges, stamp=now), now)
        self.assertGreater(nav._last_velocity[0], self.hardware.acceleration*self.hardware.control_period)
        mode, recoveries = nav.mode, nav.recoveries
        nav.pause()
        velocity, _ = nav.command(Observation(observation.pose, observation.ranges, stamp=1.1), 1.1)
        self.assertLessEqual(velocity, self.hardware.acceleration*self.hardware.control_period)
        self.assertEqual(nav.mode, mode)
        self.assertEqual(nav.recoveries, recoveries)

    def test_simulation_reaches_clear_goal_with_delays_and_noise(self):
        result = run_episode(AnalyticWorld(), self.arena, Pose(.4, .4, 0), (1.6, 1.5),
            Policy(), self.hardware, seed=27, max_time=60)
        self.assertEqual(result['status'], 'success', result)
        self.assertEqual(result['collisions'], 0)
        self.assertLessEqual(result['goal_error'], self.hardware.goal_tolerance)
        self.assertGreater(len(result['trace']), 10)

    def test_simulation_goes_around_obstacle(self):
        result = run_episode(AnalyticWorld(obstacle=(1, 1, .20)), self.arena,
            Pose(.4, 1, 0), (1.65, 1), Policy(), self.hardware,
            seed=13, max_time=100, noise=False)
        self.assertEqual(result['status'], 'success', {k: v for k, v in result.items() if k not in ('trace', 'estimated_trace', 'decisions')})
        self.assertEqual(result['collisions'], 0)
        self.assertGreater(result['mode_counts'].get('FOLLOW', 0), 0)

    def test_training_uses_holdout_only_after_selection(self):
        seeds = []
        def fake_episode(world, arena, start, goal, policy, hardware, seed=0):
            seeds.append((seed, policy.preferred_side))
            # Training prefers left; evaluation prefers right. Holdout must not
            # change selection, and every candidate has identical train seeds.
            success = policy.preferred_side == (1 if seed < 9000 else -1)
            return {'status': 'success' if success else 'timeout', 'time': 10.,
                'distance': 1., 'collisions': 0, 'goal_error': .05 if success else .5,
                'recoveries': 0, 'seed': seed, 'noise': True}
        with patch('tb3_knowledge.core.training.run_episode', side_effect=fake_episode):
            selected, report = train(AnalyticWorld(), self.arena, Pose(.4, .4), (1.6, 1.5),
                self.hardware, candidate_limit=2)
        self.assertEqual(selected.preferred_side, 1)
        self.assertEqual(report['evaluated_candidates'], 2)
        self.assertEqual(report['successes'], 2)
        self.assertEqual(report['evaluation_successes'], 0)
        self.assertEqual([seed for seed, _ in seeds[:4]], [1701, 1702, 1701, 1702])
        self.assertTrue(all(seed >= 9000 for seed, _ in seeds[4:]))

    def test_training_finishes_all_selected_sets_then_evaluates(self):
        clock=[0.]
        seeds=[]
        def episode(world,arena,start,goal,policy,hardware,seed=0):
            clock[0]+=1.;seeds.append(seed)
            return {'status':'success','time':20.,'distance':1.5,'collisions':0,
                    'goal_error':.05,'recoveries':0,'seed':seed,'noise':True}
        with patch('tb3_knowledge.core.training.time.monotonic',side_effect=lambda:clock[0]),\
             patch('tb3_knowledge.core.training.run_episode',side_effect=episode):
            _,report=train(AnalyticWorld(),self.arena,Pose(.4,.4),(1.6,1.5),self.hardware,candidate_limit=8)
        self.assertEqual(seeds,[1701,1702]*8+[9101,9102])
        self.assertEqual(report['evaluated_candidates'],8)
        self.assertEqual(report['training_seconds'],16.)
        self.assertEqual(report['total_seconds'],18.)
