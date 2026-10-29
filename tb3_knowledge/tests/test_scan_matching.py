"""Independent stationary scan localization: accuracy, symmetry and bad data."""
import math
import threading
import unittest

import numpy as np

from tb3_knowledge.core.maps import SlamMap
from tb3_knowledge.core.model import Pose,wrap
from tb3_knowledge.core.scan_matching import match_scan


def scene(symmetric=False,origin=(0.,0.,0.)):
    states=np.zeros((60,60),dtype=np.uint8)
    states[[0,-1],:]=1;states[:,[0,-1]]=1
    if not symmetric:
        states[12:21,18:24]=1;states[35:40,30:39]=1
    return SlamMap(states,.05,origin)


def measured(slam,pose,mount,count=360):
    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
    laser=Pose(pose.x+c*mount[0]-s*mount[1],pose.y+s*mount[0]+c*mount[1],pose.yaw+mount[2])
    return slam.scan(laser,count=count,max_range=8.)


class ScanMatchingTests(unittest.TestCase):
    def test_full_search_recovers_position_and_heading_without_initial_guess(self):
        slam=scene();pose=Pose(.55,.7,.4);mount=(.02,0.,0.)
        result=match_scan(slam,measured(slam,pose,mount),-math.pi,2*math.pi/360,mount)
        self.assertEqual(result.reason,'matched',result)
        self.assertLess(math.hypot(result.pose.x-pose.x,result.pose.y-pose.y),.05)
        self.assertLess(abs(wrap(result.pose.yaw-pose.yaw)),.04)

    def test_symmetric_square_does_not_choose_one_of_four_equivalent_corners(self):
        slam=scene(symmetric=True);pose=Pose(.55,.7,.4);mount=(.02,0.,0.)
        result=match_scan(slam,measured(slam,pose,mount),-math.pi,2*math.pi/360,mount)
        self.assertEqual(result.reason,'localization_ambiguous',result)
        self.assertLess(result.separation,.04)

    def test_rotated_yaml_origin_mount_and_sparse_nan_scan(self):
        slam=scene(origin=(-1.2,.8,.23))
        x,y=slam._from_image_axes(.55,.7)
        pose=Pose(x,y,.4+.23);mount=(-.032,.01,-.12)
        ranges=list(measured(slam,pose,mount,count=240))
        for index in range(0,len(ranges),9):ranges[index]=float('nan')
        result=match_scan(slam,ranges,-math.pi,2*math.pi/240,mount)
        self.assertEqual(result.reason,'matched',result)
        self.assertLess(math.hypot(result.pose.x-pose.x,result.pose.y-pose.y),.06)
        self.assertLess(abs(wrap(result.pose.yaw-pose.yaw)),.05)

    def test_invalid_partial_and_wrong_scene_cannot_be_used_as_start(self):
        slam=scene();mount=(0.,0.,0.)
        for ranges,increment in (((float('nan'),)*120,2*math.pi/120),((1.,)*120,math.pi/120)):
            with self.assertRaisesRegex(ValueError,'insufficient'):match_scan(slam,ranges,-math.pi,increment,mount)
        result=match_scan(slam,(.3,)*120,-math.pi,2*math.pi/120,mount)
        self.assertEqual(result.reason,'scan_map_mismatch')

    def test_cancelled_search_and_map_without_observed_obstacles_fail(self):
        cancel=threading.Event();cancel.set()
        with self.assertRaisesRegex(ValueError,'cancelled'):
            match_scan(scene(),(1.,)*120,-math.pi,2*math.pi/120,(0.,0.,0.),cancel=cancel)
        empty=SlamMap(np.zeros((30,30),dtype=np.uint8),.05,(0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'obstacles'):
            match_scan(empty,(1.,)*120,-math.pi,2*math.pi/120,(0.,0.,0.))

    def test_positional_hint_resolves_square_corner_without_heading_hint(self):
        slam=scene(symmetric=True);pose=Pose(.55,.7,2.1);mount=(-.032,0.,0.)
        hint=(.65,.85);radius=.30
        result=match_scan(slam,measured(slam,pose,mount),-math.pi,2*math.pi/360,mount,
            position_hint=hint,hint_radius=radius)
        self.assertEqual(result.reason,'matched',result)
        self.assertLess(math.hypot(result.pose.x-pose.x,result.pose.y-pose.y),.06)
        self.assertLess(abs(wrap(result.pose.yaw-pose.yaw)),.05)
        self.assertLessEqual(math.hypot(result.pose.x-hint[0],result.pose.y-hint[1]),radius+1e-9)

    def test_wrong_hint_does_not_silently_fall_back_to_another_corner(self):
        slam=scene();pose=Pose(.55,.7,.4);mount=(.02,0.,0.)
        hint=(2.4,2.4);radius=.2
        result=match_scan(slam,measured(slam,pose,mount),-math.pi,2*math.pi/360,mount,
            position_hint=hint,hint_radius=radius)
        self.assertNotEqual(result.reason,'matched',result)
        self.assertLessEqual(math.hypot(result.pose.x-hint[0],result.pose.y-hint[1]),radius+1e-9)

    def test_hint_uses_map_coordinates_with_rotated_origin(self):
        slam=scene(origin=(-1.2,.8,.23));px,py=slam._from_image_axes(.55,.7)
        pose=Pose(px,py,.63);mount=(.02,0.,0.)
        hint=slam._from_image_axes(.65,.8)
        result=match_scan(slam,measured(slam,pose,mount),-math.pi,2*math.pi/360,mount,
            position_hint=hint,hint_radius=.3)
        self.assertEqual(result.reason,'matched',result)
        self.assertLess(math.hypot(result.pose.x-pose.x,result.pose.y-pose.y),.06)
