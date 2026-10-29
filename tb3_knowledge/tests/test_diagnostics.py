"""Concurrent blockers and unavailable checks must not mutate the controller."""
import json
import math
import unittest
from dataclasses import replace

from tb3_knowledge.core.clearance import UnknownSector
from tb3_knowledge.core.diagnostics import movement_conditions,format_movement_conditions
from tb3_knowledge.core.model import Arena,Hardware,Policy,Pose
from tb3_knowledge.core.navigation import Navigator,Observation


class MovementConditionsTests(unittest.TestCase):
    def scene(self):
        nav=Navigator(Arena(2,2),(1.7,1.),Policy(),Hardware())
        points=tuple((.27*math.cos(i*math.pi/30),.27*math.sin(i*math.pi/30)) for i in range(60))
        obs=Observation(Pose(.5,1.),(3.,)*120,stamp=10.,points=points,
            unknown_sectors=(UnknownSector((-.032,0.),0.,.02,.16),),scan_coverage_percent=93.)
        nav.command(obs,10.)
        status=dict(reason=nav.reason,stop_latched=False,anchored=True,odom_age=.6,odom_receive_age=.7,
            scan_message_age=.05,scan_coverage_percent=93.,scan_span=2*math.pi,runtime_elapsed=12.,drive=False)
        return nav,obs,status

    def test_reports_independent_failures_with_actual_values(self):
        nav,obs,status=self.scene()
        checks=movement_conditions(status,nav,obs,10.)
        result={item['code']:item for item in checks}
        for code in ('odom_stamp','odom_delivery'):
            self.assertIs(result[code]['passed'],False,code)
        for code in ('raw_coverage','navigation_coverage','footprint','rotation','rotation_drift','forward','braking','route'):
            self.assertIs(result[code]['passed'],True,code)
        self.assertIn('0.600',result['odom_stamp']['detail'])
        self.assertIn('0.500',result['odom_stamp']['detail'])
        self.assertEqual(result['rotation']['detail'],'ไม่มีมุม unknown')
        json.dumps(checks,allow_nan=False)

    def test_stale_observation_is_unavailable_not_a_current_geometry_failure(self):
        nav,obs,status=self.scene()
        checks=movement_conditions(dict(status,stop_latched=True,reason='odometry_timeout'),nav,replace(obs,stamp=8.),10.)
        result={item['code']:item for item in checks}
        self.assertIs(result['stop_latch']['passed'],False)
        self.assertIs(result['raw_coverage']['passed'],True)
        self.assertIs(result['navigation_scan']['passed'],False)
        for code in ('rotation','forward','braking','footprint','route'):
            self.assertIsNone(result[code]['passed'])

    def test_collecting_checks_does_not_advance_or_mutate_navigation(self):
        nav,obs,status=self.scene()
        before=dict(nav.__dict__)
        movement_conditions(status,nav,obs,10.)
        self.assertEqual(nav.__dict__,before)

    def test_reports_all_guard_graph_and_loop_failures_together(self):
        nav,obs,status=self.scene()
        status.update(drive=True,odom_publisher_count=2,odom_publisher_matches=False,full_map_present=True,
            guard='waiting_motor_subscriber',guard_status_age=.8,control_loop_elapsed=.5,
            drive_armed=False,stationary_duration=.2)
        result={item['code']:item for item in movement_conditions(status,nav,None,10.)}
        for code in ('odom_source','localization_closed','guard','guard_age','control_loop','stationary'):
            self.assertIs(result[code]['passed'],False,code)

    def test_format_keeps_unavailable_separate_from_failed(self):
        checks=[dict(code=str(i),label=label,passed=passed,detail='test',scope='ระบบ')
            for i,(label,passed) in enumerate((('good',True),('bad',False),('pending',None)))]
        text=format_movement_conditions(checks,failed_only=True)
        self.assertNotIn('good',text)
        self.assertIn('[ไม่ผ่าน] ระบบ / bad',text)
        self.assertIn('[ยังตรวจไม่ได้] ระบบ / pending',text)

    def test_guard_ready_still_waits_for_initial_discovery_window(self):
        nav,obs,status=self.scene()
        status.update(drive=True,runtime_elapsed=1.,drive_armed=False,stationary_duration=1.,
            arm_pose_stable=True,arm_reference_matches=True,guard='forwarding',guard_status_age=.02,
            odom_publisher_count=1,odom_publisher_matches=True,full_map_present=False,control_loop_elapsed=.05)
        result={item['code']:item for item in movement_conditions(status,nav,obs,10.)}
        self.assertIs(result['guard']['passed'],True)
        self.assertIs(result['stationary']['passed'],True)
        self.assertIs(result['discovery_settled']['passed'],False)
