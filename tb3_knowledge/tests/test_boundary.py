"""Outer-wall detection, including rotated frames and incomplete evidence."""
import math
from pathlib import Path
import unittest

import numpy as np

from tb3_knowledge.core.maps import SlamMap,FREE,OCCUPIED,UNKNOWN


def rectangle(angle=.21,clutter=False,yaml_yaw=0):
    resolution=.025
    yy,xx=np.indices((160,160))
    x=(xx+.5)*resolution-2
    y=(160-yy-.5)*resolution-2
    c,s=math.cos(angle),math.sin(angle)
    u,v=c*x+s*y,-s*x+c*y
    inside=(abs(u)<1.2)&(abs(v)<1.)
    wall=(abs(u)<1.28)&(abs(v)<1.08)&~inside
    states=np.full(xx.shape,UNKNOWN,dtype=np.uint8)
    states[inside]=FREE;states[wall]=OCCUPIED
    if clutter:
        states[(u*u+v*v)<.32**2]=OCCUPIED
        states[(abs(u-.9)<.2)&(abs(v+.1)<.25)]=OCCUPIED
        states[2:5,2:5]=OCCUPIED  # Exterior speckle must not move the arena.
    return SlamMap(states,resolution,(.4,-.7,yaml_yaw)),u,v


class BoundaryTests(unittest.TestCase):
    def assert_rectangle(self,slam,angle):
        detected=slam.detect_arena()
        a=detected.arena
        self.assertAlmostEqual(a.width,2.4,delta=.06)
        self.assertAlmostEqual(a.height,2.,delta=.06)
        self.assertAlmostEqual(a.yaw,angle+slam.origin[2],delta=math.radians(1.2))
        center=a.local_to_map(a.width/2,a.height/2)
        np.testing.assert_allclose(center,slam._from_image_axes(2,2),atol=.04)
        self.assertGreaterEqual(min(detected.side_coverage),.6)
        self.assertGreater(detected.inlier_fraction,.65)
        return detected

    def test_axis_aligned_and_rotated_walls(self):
        for angle in (0,.21,-.34,math.radians(42)):
            with self.subTest(angle=angle):
                slam,_,_=rectangle(angle)
                self.assert_rectangle(slam,angle)

    def test_yaml_rotation_and_interior_clutter_do_not_change_frame(self):
        plain,_,_=rectangle(.21,yaml_yaw=.7)
        cluttered,_,_=rectangle(.21,clutter=True,yaml_yaw=.7)
        a=self.assert_rectangle(plain,.21).arena
        b=self.assert_rectangle(cluttered,.21).arena
        self.assertAlmostEqual(a.width,b.width,delta=.04)
        self.assertAlmostEqual(a.height,b.height,delta=.04)
        self.assertAlmostEqual(a.yaw,b.yaw,delta=math.radians(.8))

    def test_internal_unknown_is_not_the_exterior(self):
        slam,u,v=rectangle(.21)
        states=slam.states.copy()
        states[(u*u+v*v)<.3**2]=UNKNOWN
        changed=SlamMap(states,slam.resolution,slam.origin)
        self.assert_rectangle(changed,.21)

    def test_missing_wall_is_refused(self):
        slam,u,v=rectangle()
        states=slam.states.copy()
        states[(v>.95)&(abs(u)<1.15)]=UNKNOWN
        with self.assertRaises(ValueError):
            SlamMap(states,slam.resolution,slam.origin).detect_arena()

    def test_known_box_and_no_unknown_are_not_reported_as_walls(self):
        slam,_,_=rectangle()
        states=slam.states.copy();states[states==OCCUPIED]=FREE
        with self.assertRaises(ValueError):SlamMap(states,slam.resolution,slam.origin).detect_arena()
        states=slam.states.copy();states[states==UNKNOWN]=FREE
        with self.assertRaises(ValueError):SlamMap(states,slam.resolution,slam.origin).detect_arena()
        with self.assertRaises(ValueError):SlamMap(np.full((20,20),UNKNOWN),.05,(0,0,0)).detect_arena()

    def test_actual_map_read_only_and_fit_is_not_known_bbox(self):
        source=Path.home()/'tb3_maps/arena.yaml'
        if not source.exists():self.skipTest('External arena map unavailable')
        slam=SlamMap.load(source)
        before=slam.yaml_path.read_bytes(),slam.image_path.read_bytes(),slam.states.tobytes()
        result=slam.detect_arena()
        known=slam.known_arena()
        self.assertLess(result.arena.width*result.arena.height,known.width*known.height)
        self.assertGreaterEqual(min(result.side_coverage),.6)
        self.assertEqual(before,(slam.yaml_path.read_bytes(),slam.image_path.read_bytes(),slam.states.tobytes()))


if __name__=='__main__':unittest.main()
