import math
import unittest

from tb3_knowledge.core.lidar_history import Ray,Sweep,fill_missing


def sweep(*,stamp=9.9,origin=(0.,0.),count=120,missing=(),distance=1.):
    increment=2*math.pi/count
    rays=tuple(None if index in missing else Ray(origin,
        (origin[0]+distance*math.cos(-math.pi+index*increment),
         origin[1]+distance*math.sin(-math.pi+index*increment)),stamp)
        for index in range(count))
    return Sweep(rays,increment)


def to_world(pose,point):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return x+c*point[0]-s*point[1],y+s*point[0]+c*point[1]


def from_world(pose,point):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=point[0]-x,point[1]-y
    return c*dx+s*dy,-s*dx+c*dy


class LidarHistoryTests(unittest.TestCase):
    def fill(self,old,*,origin=(0.,0.),angle=0.,width=.02,points=(),now=10.):
        return fill_missing(points,[(origin,angle,width)],old,now,.5,.8)

    def test_recent_adjacent_fans_cover_complete_sector(self):
        points,unknown,oldest=self.fill([sweep()])
        self.assertEqual(unknown,[])
        self.assertEqual(oldest,9.9)
        self.assertGreater(points[0][0],.79)

    def test_centerline_alone_does_not_cover_missing_sector(self):
        # The center ray exists but neither adjacent fan exists.
        old=sweep(missing={59,61})
        points,unknown,oldest=self.fill([old])
        self.assertEqual(points,[])
        self.assertEqual(unknown,[0.])
        self.assertIsNone(oldest)

    def test_translation_never_uses_endpoint_bearing_as_free_origin(self):
        # The old free disk starts at x=.2, leaving the current sector's
        # minimum range (.16) unobserved despite a matching endpoint bearing.
        old=sweep(origin=(.3,0.),distance=.1)
        points,unknown,oldest=self.fill([old],angle=0.)
        self.assertEqual(points,[])
        self.assertEqual(unknown,[0.])
        self.assertIsNone(oldest)

    def test_persistent_holes_and_all_invalid_are_unknown(self):
        for old in ([sweep(missing={60})]*3,[sweep(missing=set(range(120)))]):
            points,unknown,oldest=self.fill(old)
            self.assertEqual(points,[])
            self.assertEqual(unknown,[0.])
            self.assertIsNone(oldest)

    def test_stale_or_future_rays_never_refresh_evidence(self):
        for stamp in (9.,10.01):
            points,unknown,oldest=self.fill([sweep(stamp=stamp)])
            self.assertEqual(points,[])
            self.assertEqual(unknown,[0.])
            self.assertIsNone(oldest)

    def test_current_close_hits_and_used_old_close_hits_survive(self):
        current=(.18,.01)
        points,unknown,oldest=self.fill([sweep(distance=.3)],points=[current])
        self.assertEqual(unknown,[])
        self.assertIn(current,points)
        self.assertTrue(any(.29<math.hypot(*point)<.31 for point in points[1:]))
        self.assertLess(points[1][0],.291)
        self.assertEqual(oldest,9.9)

    def test_circular_seam_and_negative_increment(self):
        old=sweep()
        points,unknown,oldest=self.fill([old],angle=math.pi)
        self.assertEqual(unknown,[])
        reversed_sweep=Sweep(tuple(reversed(old.rays)),-old.angle_increment)
        points,unknown,oldest=self.fill([reversed_sweep],angle=math.pi)
        self.assertEqual(unknown,[])

    def test_only_three_latest_sweeps_can_supply_evidence(self):
        old=[sweep()]+[sweep(missing={60})]*3
        points,unknown,oldest=self.fill(old)
        self.assertEqual(unknown,[0.])
        self.assertIsNone(oldest)

    def test_oldest_actually_used_evidence_timestamp(self):
        # Each history contains only one side of the sector. Both timestamps
        # are required to cover its full angular width.
        a=sweep(stamp=9.7,missing={61})
        b=sweep(stamp=9.9,missing={59})
        points,unknown,oldest=self.fill([a,b])
        self.assertEqual(unknown,[])
        self.assertEqual(oldest,9.7)

    def test_newest_sufficient_evidence_avoids_unnecessary_old_age(self):
        points,unknown,oldest=self.fill([sweep(stamp=9.6),sweep(stamp=9.9)])
        self.assertEqual(unknown,[])
        self.assertEqual(oldest,9.9)

    def test_origin_discontinuity_is_rejected(self):
        old=sweep()
        rays=list(old.rays)
        for index in (59,61):
            ray=rays[index]
            rays[index]=Ray((.02,0.),ray.endpoint,ray.stamp)
        points,unknown,oldest=self.fill([Sweep(tuple(rays),old.angle_increment)])
        self.assertEqual(unknown,[0.])

    def test_complete_sector_coverage_is_invariant_under_rigid_frame_change(self):
        # The same physical geometry seen through a translated/rotated base
        # must yield the same virtual return, including the LiDAR offset.
        pose=(.032,-.02,.73)
        old=sweep(origin=(.02,-.01),distance=.5)
        angle=.4
        initial=self.fill([old],origin=(.02,-.01),angle=angle)
        rays=tuple(Ray(to_world(pose,ray.origin),to_world(pose,ray.endpoint),ray.stamp)
                   for ray in old.rays)
        changed=self.fill([Sweep(rays,old.angle_increment)],
            origin=to_world(pose,(.02,-.01)),angle=angle+pose[2])
        self.assertEqual(initial[1],[])
        self.assertEqual(changed[1],[])
        expected=to_world(pose,initial[0][0])
        self.assertAlmostEqual(changed[0][0][0],expected[0],places=9)
        self.assertAlmostEqual(changed[0][0][1],expected[1],places=9)
        self.assertEqual(changed[2],initial[2])

    def test_changed_base_pose_compensates_sensor_origin_and_endpoint(self):
        mount=(.032,.01)
        old_base=(0.,0.,-.1)
        current_base=(.02,-.01,.23)
        old=sweep(origin=mount)
        rays=tuple(Ray(from_world(current_base,to_world(old_base,ray.origin)),
                       from_world(current_base,to_world(old_base,ray.endpoint)),ray.stamp)
                   for ray in old.rays)
        compensated=Sweep(rays,old.angle_increment)
        current_hit=(.19,.02)
        for angle in (0.,.7,math.pi):
            points,unknown,oldest=self.fill([compensated],origin=mount,
                                           angle=angle,points=[current_hit])
            self.assertEqual(unknown,[])
            self.assertEqual(points[0],current_hit)
            virtual=points[1]
            # The virtual point must agree with the current mounting origin,
            # not the older transformed sensor origin or the base center.
            delta=(virtual[0]-mount[0],virtual[1]-mount[1])
            self.assertAlmostEqual(delta[0]*math.sin(angle)-delta[1]*math.cos(angle),0.,places=9)
            self.assertGreater(math.hypot(*delta),.79)
            virtual_world=to_world(current_base,virtual)
            historical_origin=to_world(old_base,mount)
            self.assertLess(math.dist(virtual_world,historical_origin),.99)
            self.assertEqual(oldest,9.9)

    def test_motion_can_cover_annular_sector_without_covering_blind_origin(self):
        # Per-ray mounting origins change as the base moves/rotates. Coverage
        # is required from the same sensor minimum used by unknown handling;
        # the unobservable inner range is not declared free.
        count=120;increment=2*math.pi/count;mount=(.032,.01)
        current_base=(.01,0.,.1)
        rays=[]
        for index in range(count):
            fraction=index/(count-1)
            base=(.01*fraction,0.,.1*fraction)
            angle=-math.pi+index*increment
            endpoint=(mount[0]+math.cos(angle),mount[1]+math.sin(angle))
            rays.append(Ray(from_world(current_base,to_world(base,mount)),
                            from_world(current_base,to_world(base,endpoint)),9.8+.1*fraction))
        points,unknown,oldest=self.fill([Sweep(tuple(rays),increment)],origin=mount,angle=0.)
        self.assertEqual(unknown,[])
        self.assertGreater(points[0][0]-mount[0],.79)
        self.assertIsNotNone(oldest)


if __name__=='__main__':unittest.main()
