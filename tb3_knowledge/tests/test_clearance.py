"""Unknown live sectors must never open an unobserved swept-body passage."""
import math
import random
import unittest

from tb3_knowledge.core.clearance import UnknownSector,sector_distance,sector_path_clearance


class ClearanceTests(unittest.TestCase):
    def test_real_mount_distinguishes_rear_and_front_missing_data(self):
        rear=UnknownSector((-.032,0),math.pi,.02,.16)
        front=UnknownSector((-.032,0),0,.02,.16)
        self.assertGreater(sector_distance((0,0),rear),.153)
        self.assertLess(sector_distance((0,0),front),.153)
        self.assertAlmostEqual(sector_distance((0,0),rear),.192,places=4)

    def test_thin_wedge_cannot_be_skipped_between_samples(self):
        sector=UnknownSector((0,0),math.pi/2,.0001,.1)
        self.assertLess(sector_path_clearance((-.5,.5),(.5,.5),.01,sector),.5)

    def test_wide_sector_inner_chord_is_conservative(self):
        sector=UnknownSector((0,0),0,1.4,1.)
        self.assertEqual(sector_path_clearance((.4,0),(2,0),.05,sector),0.)

    def test_prefix_never_exceeds_independent_dense_contact(self):
        rng=random.Random(240)
        for _ in range(180):
            sector=UnknownSector((rng.uniform(-.2,.2),rng.uniform(-.2,.2)),
                rng.uniform(-math.pi,math.pi),rng.uniform(.001,1.4),rng.uniform(.05,.5))
            start=(rng.uniform(-.7,.7),rng.uniform(-.7,.7))
            end=(rng.uniform(-.7,.7),rng.uniform(-.7,.7));radius=rng.uniform(.01,.2)
            prefix=sector_path_clearance(start,end,radius,sector)
            length=math.dist(start,end)
            self.assertTrue(0<=prefix<=length+1e-10)
            for i in range(601):
                t=i/600
                point=(start[0]+t*(end[0]-start[0]),start[1]+t*(end[1]-start[1]))
                if sector_distance(point,sector)<=radius:
                    self.assertLessEqual(prefix,t*length+1e-8)
                    break
