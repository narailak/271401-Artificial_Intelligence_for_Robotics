"""A newly arriving sweep must not starve the previous completed sweep."""
from collections import deque
import math
from types import SimpleNamespace
import unittest

from tb3_knowledge.core.model import Hardware, Pose
from tb3_knowledge.core.ros import PoseBuffer, aligned_scan_observation, complete_scan, scan_coverage


def scan(stamp,duration=.1):
    seconds=int(stamp)
    header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds,nanosec=round((stamp-seconds)*1e9)))
    return SimpleNamespace(header=header,ranges=[1.]*5,time_increment=duration/4)


class ScanCoverageTests(unittest.TestCase):
    def test_counts_nonfinite_and_out_of_range_readings_as_missing(self):
        message=SimpleNamespace(range_min=.16,range_max=8.,ranges=(.16,1.,8.,float('nan'),float('inf'),-1.,.1,8.1))
        self.assertEqual(scan_coverage(message),dict(scan_total_beams=8,scan_valid_beams=3,
            scan_missing_beams=5,scan_coverage_percent=37.5,scan_missing_percent=62.5))

    def test_no_scan_has_no_score_instead_of_false_full_confidence(self):
        for message in (None,SimpleNamespace(ranges=())):
            self.assertIsNone(scan_coverage(message)['scan_coverage_percent'])
            self.assertIsNone(scan_coverage(message)['scan_missing_percent'])

    def test_all_missing_and_complete_scans_have_opposite_scores(self):
        for ranges,expected in (((float('nan'),)*10,0.),((1.,)*10,100.)):
            result=scan_coverage(SimpleNamespace(ranges=ranges,range_min=.16,range_max=8.))
            self.assertEqual(result['scan_coverage_percent'],expected)
            self.assertEqual(result['scan_missing_percent'],100-expected)


class ScanTimingTests(unittest.TestCase):
    def setUp(self):
        self.poses=PoseBuffer()
        self.poses.add(10.,Pose(0,0))
        self.poses.add(11.,Pose(0,0))

    def test_latest_incomplete_sweep_does_not_replace_usable_previous(self):
        previous,newest=scan(10.8),scan(10.95)
        history=deque([previous,newest],maxlen=32)
        self.assertIs(complete_scan(history,self.poses,11.02,.6),previous)
        self.poses.add(11.1,Pose(0,0))
        self.assertIs(complete_scan(history,self.poses,11.12,.6),newest)

    def test_waits_for_actual_odometry_without_extrapolating(self):
        with self.assertRaisesRegex(ValueError,'waiting_scan_odometry'):
            complete_scan([scan(10.95)],self.poses,11.02,.6)

    def test_rejects_complete_but_stale_sweep(self):
        with self.assertRaisesRegex(ValueError,'stale_or_future_scan'):
            complete_scan([scan(10.1)],self.poses,11.02,.6)

    def test_requires_history_at_first_beam_too(self):
        with self.assertRaisesRegex(ValueError,'odometry_history_missing'):
            complete_scan([scan(9.95)],self.poses,10.1,.6)

    def test_uses_newest_complete_sweep(self):
        old,new=scan(10.7),scan(10.85)
        self.assertIs(complete_scan([old,new],self.poses,11.02,.6),new)

    def test_waits_for_initial_sensors(self):
        with self.assertRaisesRegex(ValueError,'waiting_scan'):
            complete_scan([],self.poses,11.,.6)
        with self.assertRaisesRegex(ValueError,'waiting_odometry'):
            complete_scan([scan(10.9)],PoseBuffer(),11.,.6)

    def test_batch_interpolation_matches_single_at_including_yaw_wrap(self):
        poses=PoseBuffer()
        poses.add(10.,Pose(0,0,math.pi-.1))
        poses.add(11.,Pose(.1,.2,-math.pi+.1))
        stamps=[10.,10.1,10.5,10.9,11.]
        for actual,expected in zip(poses.at_many(stamps),[poses.at(t,0) for t in stamps]):
            self.assertAlmostEqual(actual.x,expected.x)
            self.assertAlmostEqual(actual.y,expected.y)
            self.assertAlmostEqual(actual.yaw,expected.yaw)
        with self.assertRaisesRegex(ValueError,'history_missing'):poses.at_many([9.99])
        with self.assertRaisesRegex(ValueError,'waiting_scan_odometry'):poses.at_many([11.01])

    def test_live_history_fills_nan_without_refreshing_its_timestamp(self):
        def full(stamp,missing=()):
            result=scan(stamp,.08)
            result.ranges=[float('nan') if i in missing else 1. for i in range(120)]
            result.time_increment=.08/119
            result.angle_min=-math.pi
            result.angle_increment=2*math.pi/120
            result.range_min=.16;result.range_max=8.
            result.header.frame_id='laser'
            return result
        previous,current=full(10.7),full(10.85,{60})
        obs=aligned_scan_observation(current,[previous,current],self.poses,Pose(.5,.5),0.,
            (0.,0.,0.),11.02,Hardware())
        self.assertEqual(obs.invalid_angles,())
        self.assertLess(obs.stamp,10.85)
        self.assertGreaterEqual(obs.beam_spacing,2*abs(current.angle_increment))
        self.assertEqual(sum(math.isnan(x) for x in obs.ranges),1)
        # A persistent hole stays unknown; future sweeps cannot fill it.
        previous=full(10.7,{60})
        future=full(10.95)
        obs=aligned_scan_observation(current,[previous,current,future],self.poses,Pose(.5,.5),0.,
            (0.,0.,0.),11.02,Hardware())
        self.assertEqual(obs.invalid_angles,(0.,))
        self.assertEqual(obs.stamp,10.85)
        # History must not replace another kind of invalid measurement.
        current.ranges[60]=float('inf')
        previous=full(10.7)
        obs=aligned_scan_observation(current,[previous,current],self.poses,Pose(.5,.5),0.,
            (0.,0.,0.),11.02,Hardware())
        self.assertEqual(obs.invalid_angles,(0.,))
        self.assertEqual(obs.stamp,10.85)

    def test_deskew_transforms_lidar_origin_and_endpoint_at_each_ray(self):
        poses=PoseBuffer()
        poses.add(10.,Pose(0,0,0))
        poses.add(11.,Pose(.1,0,.1))
        current=scan(10.,1.)
        current.ranges=[1.]*120;current.time_increment=1/119
        current.angle_min=-math.pi;current.angle_increment=2*math.pi/120
        current.range_min=.16;current.range_max=8.;current.header.frame_id='laser'
        obs=aligned_scan_observation(current,[current],poses,Pose(.5,.5),0.,(.03,0.,0.),11.,Hardware())
        # First ray (-1,0) from the old mounted origin (.03,0), transformed
        # into the final base frame translated by .1 and rotated by .1 rad.
        self.assertAlmostEqual(obs.points[0][0],-1.07*math.cos(.1))
        self.assertAlmostEqual(obs.points[0][1],1.07*math.sin(.1))

    def test_narrow_dropout_filter_preserves_raw_quality_and_inflates_spacing(self):
        current=scan(10.85,.08)
        current.ranges=[1.]*360;current.ranges[180]=float('nan')
        current.time_increment=.08/359
        current.angle_min=-math.pi;current.angle_increment=2*math.pi/360
        current.range_min=.16;current.range_max=8.;current.header.frame_id='laser'
        obs=aligned_scan_observation(current,[current],self.poses,Pose(.5,.5),0.,(0.,0.,0.),11.02,Hardware())
        self.assertEqual(obs.interpolated_beams,1)
        self.assertEqual(obs.invalid_angles,())
        self.assertAlmostEqual(obs.scan_coverage_percent,100*359/360)
        self.assertTrue(math.isnan(obs.ranges[180]))
        self.assertAlmostEqual(obs.beam_spacing,2*current.angle_increment)
        self.assertAlmostEqual(obs.points[180][0],.98)

    def test_clipped_seam_uses_only_fresh_recorded_coverage(self):
        def full(stamp,count=120,missing=()):
            result=scan(stamp,.08)
            result.ranges=[float('nan') if i in missing else 1. for i in range(count)]
            result.time_increment=.08/(count-1)
            result.angle_min=-math.pi;result.angle_increment=2*math.pi/120
            result.range_min=.16;result.range_max=8.;result.header.frame_id='laser'
            return result
        current=full(10.85,119)
        for history in ([],[full(10.7,missing=(118,119))],[full(10.1)],[full(10.95)]):
            obs=aligned_scan_observation(current,history+[current],self.poses,Pose(.5,.5),0.,
                (-.032,0.,0.),11.02,Hardware())
            self.assertEqual(len(obs.unknown_sectors),1)
            self.assertEqual(obs.stamp,10.85)
        obs=aligned_scan_observation(current,[full(10.7),current],self.poses,Pose(.5,.5),0.,
            (-.032,0.,0.),11.02,Hardware())
        self.assertEqual(obs.unknown_sectors,())
        self.assertLess(obs.stamp,10.85)

    def test_clockwise_deskew_includes_widened_last_to_first_seam(self):
        poses=PoseBuffer()
        poses.add(10.,Pose(0,0,0))
        poses.add(11.,Pose(0,0,-.1))
        current=scan(10.,1.)
        current.ranges=[1.]*120;current.time_increment=1/119
        current.angle_min=-math.pi;current.angle_increment=2*math.pi/120
        current.range_min=.16;current.range_max=8.;current.header.frame_id='laser'
        obs=aligned_scan_observation(current,[current],poses,Pose(.5,.5),0.,(0.,0.,0.),11.,Hardware())
        # Only the seam widens. Inflating every ray by this seam would also
        # invent obstacle thickness at unrelated angles and close real gaps.
        self.assertAlmostEqual(obs.beam_spacing,current.angle_increment)
        self.assertEqual(len(obs.unknown_sectors),1)
        self.assertAlmostEqual(2*obs.unknown_sectors[0].half_width,current.angle_increment+.1)
        self.assertEqual(obs.angle_increment,current.angle_increment)
