"""Sensor-only Knowledge-Guided Gap Navigation.

This module deliberately imports neither the SLAM map nor Nav2.  Surveyed
obstacles never reach Navigator; its temporary memory is created during a run.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from .model import Arena, Hardware, Policy, Pose
from .clearance import UnknownSector,sector_distance,sector_path_clearance

MIN_SCAN_COVERAGE_PERCENT = 90.0


def wrap(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Observation:
    pose: Pose
    ranges: tuple[float, ...]
    angle_min: float = -math.pi
    angle_increment: float | None = None
    stamp: float = 0.0
    mount: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_speed: float = 0.0
    # Runtime may supply per-ray deskewed endpoints in the *current* base frame.
    points: tuple[tuple[float, float], ...] | None = None
    invalid_angles: tuple[float, ...] | None = None
    # Deskew/history may increase angular uncertainty, never reduce it.
    beam_spacing: float | None = None
    unknown_sectors: tuple[UnknownSector,...] | None = None
    angular_speed: float = 0.0
    scan_coverage_percent: float | None = None
    interpolated_beams: int = 0

    def sensor_points(self, maximum: float) -> tuple[list[tuple[float, float]], list[float]]:
        if self.points is not None:
            return list(self.points), list(self.invalid_angles or ())
        points, invalid = [], []
        increment = self.angle_increment
        if increment is None:
            increment = 2 * math.pi / max(1, len(self.ranges))
        mx, my, yaw = self.mount
        for index, value in enumerate(self.ranges):
            angle = self.angle_min + index * increment + yaw
            try:
                valid = math.isfinite(value) and 0 < value <= maximum + 0.05
            except TypeError:
                valid = False
            if not valid:
                invalid.append(wrap(angle))
                continue
            points.append((mx + value * math.cos(angle), my + value * math.sin(angle)))
        return points, invalid


class Navigator:
    """Gap scoring with persistent bypass side, progress recovery and hard filters.

    Coordinates are arena-local metres. Every obstacle endpoint is live sensor
    data. Neither a global route nor a spatial obstacle prior is accepted.
    """

    def __init__(self, arena: Arena, goal: tuple[float, float], policy: Policy, hardware: Hardware):
        self.arena, self.goal, self.policy, self.hardware = arena, tuple(goal), policy, hardware
        self.mode = "GOAL"
        self.reason = "ready"
        self.local_path: list[tuple[float, float]] = []
        self.recoveries = 0
        self._side = 1 if policy.preferred_side >= 0 else -1
        self._last_heading = 0.0
        self._last_world_heading: float | None = None
        self._last_time: float | None = None
        self._last_velocity = (0.0, 0.0)
        self._poses: deque[tuple[float, float, float]] = deque(maxlen=1000)
        self._progress: deque[tuple[float, float]] = deque()
        self._mode_since = 0.0
        self._hit_distance = math.inf
        self._terminal = False
        self._beam_width = 2 * math.pi / hardware.scan_beams
        self._scan_advance_state = None
        self.last_checked_observation = None
        self.candidate_count = None
        self.last_arc_clear = None
        self.last_arc_scope = 'เดินปกติ'

    @property
    def radius(self) -> float:
        return self.hardware.radius + self.hardware.safety_margin

    def pause(self) -> None:
        """Reset command ramp after an external forced zero without forgetting policy state."""
        self._last_velocity = (0.0, 0.0)
        self._last_time = None
        self.local_path = []

    def _inside(self, x: float, y: float) -> bool:
        radius = self.radius
        return radius <= x <= self.arena.width - radius and radius <= y <= self.arena.height - radius

    def _stop(self, reason: str, terminal: bool = False) -> tuple[float, float]:
        self.reason = reason
        self.local_path = []
        self._last_velocity = (0.0, 0.0)
        self._terminal = self._terminal or terminal
        return 0.0, 0.0

    def _clearance(self, heading: float, pose: Pose, points, invalid, length: float) -> float:
        """Length a swept circle can travel on a ray, before any hard constraint."""
        cosine, sine = math.cos(heading), math.sin(heading)
        clearance = length
        for px, py in points:
            # A wall edge can lie anywhere between this return and an adjacent
            # ray. Reserve one full angular spacing, not half a beam: the next
            # ray may pass through an opening while this one lands on its edge.
            radius = self.radius + math.hypot(px, py) * math.sin(self._beam_width)
            along = px * cosine + py * sine
            across = abs(-px * sine + py * cosine)
            if across <= radius:
                root = math.sqrt(max(0.0, radius * radius - across * across))
                if along + root >= 0:
                    clearance = min(clearance, max(0.0, along - root))
        # An invalid ray is unknown from the sensor minimum range onwards.
        # Reserve a half beam at each side; missing rays never form a fake gap.
        radius = self.radius
        for sector in invalid:
            if not isinstance(sector,UnknownSector):
                sector=UnknownSector((0.,0.),sector,self._beam_width/2,self.hardware.lidar_min)
            clearance=min(clearance,sector_path_clearance((0.,0.),(length*cosine,length*sine),radius,sector))
        direction = pose.yaw + heading
        dx, dy = math.cos(direction), math.sin(direction)
        if dx > 1e-8:
            clearance = min(clearance, (self.arena.width - radius - pose.x) / dx)
        elif dx < -1e-8:
            clearance = min(clearance, (radius - pose.x) / dx)
        if dy > 1e-8:
            clearance = min(clearance, (self.arena.height - radius - pose.y) / dy)
        elif dy < -1e-8:
            clearance = min(clearance, (radius - pose.y) / dy)
        return max(0.0, clearance)

    def _arc_clear(self, pose: Pose, points, velocity: float, angular: float, duration: float, invalid=()) -> bool:
        """Check the commanded curved trajectory, not only its nominal heading."""
        cy, sy = math.cos(pose.yaw), math.sin(pose.yaw)
        previous=(0.,0.)
        for index in range(1, 7):
            elapsed = duration * index / 6
            if abs(angular) < 1e-8:
                x, y = velocity * elapsed, 0.0
            else:
                x = velocity / angular * math.sin(angular * elapsed)
                y = velocity / angular * (1 - math.cos(angular * elapsed))
            if not self._inside(pose.x + cy * x - sy * y, pose.y + sy * x + cy * y):
                return False
            if any((px - x) ** 2 + (py - y) ** 2 <
                   (self.radius + math.hypot(px, py) * math.sin(self._beam_width)) ** 2
                   for px, py in points):
                return False
            # The chord capsule plus an arc sagitta bound covers the whole
            # curved interval, not just six sampled centers.
            sagitta=abs(velocity*angular)*(duration/6)**2/8
            for sector in invalid:
                if sector_path_clearance(previous,(x,y),self.radius+sagitta,sector)<math.dist(previous,(x,y))-1e-9:
                    return False
                if sector_distance((x,y),sector)<=self.radius+sagitta:return False
            previous=(x,y)
        return True

    def _advance_for_scan(self, observation, now, points, invalid, dt):
        """Use a proved short corridor to change viewpoint without turning."""
        if not invalid:
            return None
        h,pose=self.hardware,observation.pose
        clearance=self._clearance(0.,pose,points,invalid,.13)
        if self._scan_advance_state is None:
            budget=min(.05,clearance-.025)
            if budget<.02:
                return None
            started,travelled=now,0.
        else:
            started,x,y,travelled,budget=self._scan_advance_state
            travelled+=math.hypot(pose.x-x,pose.y-y)
            self._scan_advance_state=(started,pose.x,pose.y,travelled,budget)
        remaining=budget-travelled
        if remaining<=.005 or now-started>=5.:
            return self._stop('waiting_scan_after_advance')
        latency=max(0.,now-observation.stamp)+h.command_timeout+h.control_period
        space=min(remaining-.005,clearance-.025)
        speed=max(0.,observation.linear_speed)
        if abs(observation.angular_speed)>.025 or space<=0 or speed*latency+speed*speed/(2*h.braking)>space:
            return self._stop('waiting_scan_clearance')
        braking_speed=math.sqrt((h.braking*latency)**2+2*h.braking*space)-h.braking*latency
        velocity=min(.02,h.max_speed,braking_speed,max(0.,self._last_velocity[0])+h.acceleration*dt)
        duration=latency+max(speed,velocity)/h.braking
        self.last_arc_clear=self._arc_clear(pose,points,velocity,0.,duration,invalid)
        self.last_arc_scope='ก้าวตรวจ'
        if not self.last_arc_clear:
            return self._stop('waiting_scan_clearance')
        # Keep the measured distance/time budget across pauses and new scans.
        self._scan_advance_state=(started,pose.x,pose.y,travelled,budget)
        self._last_velocity=(velocity,0.)
        self.mode='RECOVER';self.reason='advancing_for_scan'
        self.local_path=[(pose.x,pose.y),(pose.x+remaining*math.cos(pose.yaw),pose.y+remaining*math.sin(pose.yaw))]
        return velocity,0.

    def rotation_radius(self, observation, now):
        """Physical rotation envelope plus 2 mm and residual translation to stop."""
        speed=abs(observation.linear_speed)
        latency=max(0.,now-observation.stamp)+self.hardware.command_timeout+self.hardware.control_period
        return self.hardware.radius+.002+speed*latency+speed*speed/(2*self.hardware.braking)

    def _observe_for_gap(self, observation, now, points, invalid, dt, heading):
        # An in-place turn sweeps the physical body circle, not the translation padding.
        radius=self.rotation_radius(observation,now)
        if any(sector_distance((0.,0.),sector)<=radius for sector in invalid):
            return self._stop('unknown_rotation_clearance')
        drift=radius-self.hardware.radius-.002
        pose=observation.pose
        if (not self.arena.contains(pose.x,pose.y,self.radius+drift)
                or any(math.hypot(x,y)<self.radius+drift+math.hypot(x,y)*math.sin(self._beam_width) for x,y in points)):
            return self._stop('rotation_drift_clearance')
        angular=math.copysign(min(.15,self.hardware.max_angular_speed),heading if abs(heading)>.15 else self._side)
        angular=_clip(angular,self._last_velocity[1]-self.hardware.angular_acceleration*dt,
                      self._last_velocity[1]+self.hardware.angular_acceleration*dt)
        angular=_clip(angular,-.15,.15)
        self._last_velocity=(0.,angular)
        self.mode='RECOVER';self.reason='observing_for_gap'
        self.last_arc_clear=True;self.last_arc_scope='หมุน'
        self.local_path=[(pose.x,pose.y)]
        return 0.,angular

    def command(self, observation: Observation, now: float) -> tuple[float, float]:
        self.last_checked_observation=observation
        self.candidate_count=None
        self.last_arc_clear=None
        if self._terminal:
            return self._stop(self.reason)
        h, p, pose = self.hardware, self.policy, observation.pose
        if not all(math.isfinite(v) for v in (now, observation.stamp, pose.x, pose.y, pose.yaw, observation.linear_speed, observation.angular_speed)):
            return self._stop("invalid_observation")
        if observation.scan_coverage_percent is not None and (not math.isfinite(observation.scan_coverage_percent)
                or not 0<=observation.scan_coverage_percent<=100):
            return self._stop('invalid_observation')
        age = now - observation.stamp
        if age > h.scan_timeout or age < -0.06:
            return self._stop("stale_scan" if age >= 0 else "future_scan")
        if len(observation.ranges) < 16 and observation.points is None:
            return self._stop("incomplete_scan")
        if observation.angle_increment is not None and (not math.isfinite(observation.angle_increment) or observation.angle_increment == 0):
            return self._stop("invalid_scan_angles")
        if observation.beam_spacing is not None and (not math.isfinite(observation.beam_spacing) or not 0 < observation.beam_spacing < math.pi/2):
            return self._stop("invalid_scan_angles")
        if (not math.isfinite(observation.angle_min) or len(observation.mount) != 3
                or not all(math.isfinite(v) for v in observation.mount)
                or observation.invalid_angles is not None and not all(math.isfinite(v) for v in observation.invalid_angles)):
            return self._stop("invalid_scan_angles")
        if not self._inside(pose.x, pose.y):
            return self._stop("outside_safe_arena", terminal=True)
        distance = math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y)
        # Reserve 20% of the requested physical tolerance for estimation error.
        # This is a stopping buffer, not a guarantee about real odometry drift.
        if distance <= 0.8 * h.goal_tolerance:
            self.mode = "ARRIVED"
            return self._stop("arrived_estimate", terminal=True)
        points, invalid = observation.sensor_points(h.scan_range)
        if not points or any(not math.isfinite(x) or not math.isfinite(y) for x, y in points):
            return self._stop("invalid_scan")
        # Missing angular coverage is unknown. Reject partial scans explicitly;
        # deskewed runtime observations carry the original ranges as coverage.
        increment = observation.angle_increment or (2 * math.pi / max(1, len(observation.ranges)))
        if not 2 * math.pi - 0.15 <= len(observation.ranges) * abs(increment) <= 2 * math.pi + 0.15:
            return self._stop("incomplete_scan")
        self._beam_width = max(abs(increment),observation.beam_spacing or 0.)
        if observation.unknown_sectors is not None:
            if any(not isinstance(sector,UnknownSector) for sector in observation.unknown_sectors):
                return self._stop('invalid_scan_angles')
            invalid=list(observation.unknown_sectors)
        else:
            invalid=[UnknownSector(observation.mount[:2],angle,self._beam_width/2,h.lidar_min) for angle in invalid]
        if not invalid:
            self._scan_advance_state=None
        if any(x * x + y * y < (self.radius + math.hypot(x, y) * math.sin(self._beam_width)) ** 2 - 1e-6 for x, y in points):
            return self._stop("footprint_clearance")
        coverage=observation.scan_coverage_percent
        if coverage is None:
            coverage=100*sum(math.isfinite(value) and 0<value<=h.scan_range+.05 for value in observation.ranges)/len(observation.ranges)
        if coverage<MIN_SCAN_COVERAGE_PERCENT:
            return self._stop('insufficient_scan_coverage')
        # Low-padding mode: once raw coverage passes, navigate from measured
        # returns. Missing rays no longer become virtual near-body obstacles.
        invalid=[]
        self._scan_advance_state=None
        dt = h.control_period if self._last_time is None else _clip(now - self._last_time, 0.001, 0.2)
        self._last_time = now
        self._poses.append((now, pose.x, pose.y))
        self._progress.append((now, distance))
        while self._progress and now - self._progress[0][0] > p.progress_window:
            self._progress.popleft()
        if self._progress and now - self._progress[0][0] >= p.progress_window - dt * 2:
            progress = self._progress[0][1] - min(d for _, d in self._progress)
            if progress < p.min_progress:
                self.recoveries += 1
                if self.recoveries > 4:
                    self.mode = "STOPPED"
                    return self._stop("recovery_exhausted", terminal=True)
                self.mode = "RECOVER"
                self._side *= -1
                self._mode_since = now
                self._progress.clear()
                self._hit_distance = distance
        if self._last_world_heading is not None:
            # Use measured orientation change, including actuator latency and
            # odometry noise; predicting yaw from the last command is unsafe.
            self._last_heading = wrap(self._last_world_heading - pose.yaw)
        goal_heading = wrap(math.atan2(self.goal[1] - pose.y, self.goal[0] - pose.x) - pose.yaw)
        horizon = max(0.2, p.lookahead)
        direct_clearance = self._clearance(goal_heading, pose, points, invalid, horizon)
        direct_ok = direct_clearance >= min(horizon * 0.95, max(0.05, distance - h.goal_tolerance))
        if self.mode == "GOAL" and not direct_ok:
            self.mode, self._mode_since, self._hit_distance = "FOLLOW", now, distance
        elif self.mode in ("FOLLOW", "RECOVER") and direct_ok and now - self._mode_since >= p.side_hold and distance < self._hit_distance - p.min_progress:
            self.mode, self._mode_since = "GOAL", now

        # Candidate spacing plus exact desired/previous heading. Capsule checks
        # reject passages narrower than the whole robot before cost evaluation.
        headings = [wrap(-math.pi + i * math.pi / 18) for i in range(36)]
        headings.extend((0.0, goal_heading, wrap(self._last_heading)))
        candidates = []
        for heading in headings:
            clearance = self._clearance(heading, pose, points, invalid, horizon)
            # Incomplete coverage keeps the bounded scan-collection fallback.
            minimum_clearance = 0.13 if invalid else 0.05
            if clearance < min(minimum_clearance, max(0.05, distance - h.goal_tolerance)):
                continue
            target_x = pose.x + min(clearance, 0.35) * math.cos(pose.yaw + heading)
            target_y = pose.y + min(clearance, 0.35) * math.sin(pose.yaw + heading)
            revisits = sum(math.hypot(target_x - x, target_y - y) < 0.13 for stamp, x, y in list(self._poses)[::8] if now - stamp > 4)
            revisit_cost = min(1.0, revisits / 8)
            goal_cost = abs(wrap(heading - goal_heading)) / math.pi
            clearance_cost = 1 - clearance / horizon
            turn_cost = abs(wrap(heading - self._last_heading)) / math.pi
            cost = p.goal_weight * goal_cost + p.clearance_weight * clearance_cost + p.turn_weight * turn_cost + p.revisit_weight * revisit_cost
            if self.mode in ("FOLLOW", "RECOVER"):
                # A global scalar chooses a persistent bypass side. No obstacle
                # coordinates or precomputed waypoints influence this term.
                side_delta = wrap(heading - goal_heading)
                if self._side * side_delta < -0.03:
                    cost += 1.25 if self.mode == "FOLLOW" else 2.0
                cost += 0.20 * abs(clearance - min(horizon, p.wall_distance * 2)) / horizon
            candidates.append((cost, heading, clearance))
        self.candidate_count=len(candidates)
        if not candidates:
            advance=self._advance_for_scan(observation,now,points,invalid,dt)
            if advance is not None:
                return advance
            # A circular robot may inspect another direction in place even
            # when no translational passage is currently proved. Unknown
            # sectors intersecting its actual swept circle still forbid this.
            return self._observe_for_gap(observation,now,points,invalid,dt,goal_heading)
        candidates.sort()
        best = candidates[0]
        previous = min(candidates, key=lambda item: abs(wrap(item[1] - self._last_heading)))
        if previous[0] <= best[0] + p.hysteresis:
            best = previous
        _, heading, clearance = best
        angular = _clip(2.0 * heading, -h.max_angular_speed, h.max_angular_speed)
        front_clearance = self._clearance(0.0, pose, points, invalid, horizon)
        latency = max(0, age) + h.command_timeout + h.control_period
        stopping_space = max(0.0, front_clearance - 0.025)
        braking_speed = max(0.0, math.sqrt((h.braking * latency) ** 2 + 2 * h.braking * stopping_space) - h.braking * latency)
        velocity = min(h.max_speed, braking_speed, h.max_speed * max(0.0, front_clearance / max(0.01, p.slowdown_distance)), distance)
        velocity *= max(0.0, math.cos(heading)) ** 2
        if abs(heading) > 0.45:
            velocity = 0.0
        # Rotation has a circular footprint. Unknown nearby sectors do not
        # authorize blind rotation; a real endpoint inside the circle stops all.
        if any(sector_distance((0.,0.),sector)<=self.radius for sector in invalid):
            if abs(angular) > 0.01:
                advance=self._advance_for_scan(observation,now,points,invalid,dt)
                if advance is not None:
                    return advance
                return self._observe_for_gap(observation,now,points,invalid,dt,heading)
        previous_v, previous_w = self._last_velocity
        velocity = min(velocity, max(0.0, previous_v) + h.acceleration * dt)
        angular = _clip(angular, previous_w - h.angular_acceleration * dt, previous_w + h.angular_acceleration * dt)
        duration = latency + max(observation.linear_speed, velocity) / h.braking
        self.last_arc_clear=self._arc_clear(pose,points,velocity,angular,duration,invalid)
        self.last_arc_scope='เดินปกติ'
        if not self.last_arc_clear:
            velocity = 0.0
        self._last_heading = heading
        self._last_world_heading = wrap(pose.yaw + heading)
        self._last_velocity = (velocity, angular)
        self.reason = "navigating"
        self.local_path = [(pose.x, pose.y), (pose.x + min(clearance, horizon) * math.cos(pose.yaw + heading), pose.y + min(clearance, horizon) * math.sin(pose.yaw + heading))]
        return velocity, angular
