"""Queued old odometry is discarded; it cannot arm or sustain the controller."""
import unittest

from tb3_knowledge.core.model import Arena,InitialReference,Knowledge,Pose
from tb3_knowledge.core.ros import RuntimeState


class RuntimeTimingTests(unittest.TestCase):
    def state(self):
        reference=InitialReference('localized','tb3','odom','base_footprint',Pose(0,0),10.,10.)
        return RuntimeState(Knowledge(Arena(2,2),Pose(.4,.4),(1.4,1.4),reference=reference))

    def odometry(self,state,stamp,now,monotonic):
        return state.odometry(stamp,Pose(0,0),'odom','base_footprint',0.,0.,now,monotonic)

    def test_old_startup_sample_does_not_anchor_then_fresh_sample_works(self):
        state=self.state()
        self.assertFalse(self.odometry(state,10.,11.,1.))
        self.assertFalse(state.anchored)
        self.assertFalse(state.buffer.samples)
        self.assertEqual(state.odom_seen,0.)
        self.assertEqual(state.stopped,'')
        self.assertTrue(self.odometry(state,11.,11.02,1.1))
        self.assertTrue(state.anchored)

    def test_repeated_old_samples_cannot_keep_watchdog_alive(self):
        state=self.state()
        self.odometry(state,11.,11.01,1.)
        state.scan_seen=1.6
        for clock in (11.5,11.6):
            self.assertFalse(self.odometry(state,10.8,clock,1.+clock-11.))
        self.assertEqual(state.odom_seen,1.)
        self.assertEqual(state.health(11.6,1.6),'odometry_timeout')

    def test_future_sample_still_rejected(self):
        with self.assertRaisesRegex(ValueError,'stale_or_future_odometry'):
            self.odometry(self.state(),11.1,11.,1.)

    def test_half_second_timeout_accepts_point_43_but_rejects_point_51(self):
        state=self.state()
        self.assertEqual(state.hardware.odom_timeout,.5)
        self.assertTrue(self.odometry(state,11.,11.43,1.))
        state.scan_seen=1.43
        self.assertEqual(state.health(11.43,1.43),'')
        self.assertEqual(state.health(11.51,1.51),'odometry_timeout')

    def test_invalid_stamp_is_not_a_harmless_dropout(self):
        for stamp in (0.,float('nan'),float('inf')):
            with self.assertRaisesRegex(ValueError,'invalid_odometry_stamp'):
                self.odometry(self.state(),stamp,11.,1.)
