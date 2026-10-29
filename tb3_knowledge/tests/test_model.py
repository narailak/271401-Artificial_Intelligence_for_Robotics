"""Knowledge contract and initial-reference tests, independent of maps/ROS."""
from dataclasses import replace
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.model import Arena,Hardware,InitialReference,Knowledge,Policy,Pose,digest


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.k=Knowledge(Arena(2.5,2.5,1.2,-.7,.3),Pose(.4,.4,.7),(2.,2.))

    def test_rotated_coordinates_roundtrip(self):
        p=Pose(.6,1.2,-.4)
        mapped=self.k.arena.pose_to_map(p)
        restored=self.k.arena.pose_from_map(mapped)
        for a,b in zip((p.x,p.y,p.yaw),(restored.x,restored.y,restored.yaw)):
            self.assertAlmostEqual(a,b)

    def test_burger_radius_covers_offset_body_corners(self):
        hardware=Hardware()
        for x in (-.032-.140/2,-.032+.140/2):
            for y in (-.140/2,.140/2):
                self.assertGreaterEqual(hardware.radius,math.hypot(x,y))

    def test_single_file_roundtrip_and_no_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            file=self.k.save(Path(tmp)/'knowledge.json')
            loaded=Knowledge.load(file)
            self.assertEqual(loaded,self.k)
            self.assertEqual(loaded.fingerprint,self.k.fingerprint)
            self.assertEqual([p.name for p in Path(tmp).iterdir()],['knowledge.json'])
            for key in ('obstacles','occupancy','waypoints','source_map','path'):
                self.assertNotIn(key,json.dumps(loaded.document()))

    def test_reject_tampering_and_geometry_even_with_recomputed_fingerprint(self):
        data=self.k.document();data['mission']['goal'][0]=1.8
        with self.assertRaises(ValueError):Knowledge.from_document(data)
        data=self.k.document();data['mission']['obstacles']=[]
        data['fingerprint']=digest({k:v for k,v in data.items() if k!='fingerprint'})
        with self.assertRaises(ValueError):Knowledge.from_document(data)
        data=self.k.document();data['training_summary']={'obstacles':2}
        data['fingerprint']=digest({k:v for k,v in data.items() if k!='fingerprint'})
        with self.assertRaises(ValueError):Knowledge.from_document(data)

    def test_reference_types_and_finite_values(self):
        for kwargs in ({'mode':'bad'},{'mode':'localized'},{'odom_frame':'odom','base_frame':'odom'}, {'stamp':float('nan')}):
            with self.assertRaises(ValueError):InitialReference(**kwargs)
        ref=InitialReference(mode='localized',robot_id='robot2',odom_pose=Pose(1,2,.1),stamp=5,captured_at=6)
        k=replace(self.k,reference=ref)
        self.assertEqual(Knowledge.from_document(k.document()),k)
        for value in (float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):Pose(value,0)

    def test_reject_unsafe_bounds_hardware_and_summary(self):
        for goal in ((0,1),(2.5,1),(1,float('nan'))):
            with self.assertRaises(ValueError):replace(self.k,goal=goal)
        for kwargs in ({'safety_margin':0},{'max_speed':1},{'scan_beams':20}):
            with self.assertRaises(ValueError):Hardware(**kwargs)
        with self.assertRaises(ValueError):replace(self.k,training_summary={'cases':{'x':1}})
        with self.assertRaises(ValueError):Policy(preferred_side=0)

    def test_program2_inspect_without_maps_or_ros(self):
        script=Path(__file__).resolve().parents[1]/'run_knowledge.py'
        with tempfile.TemporaryDirectory() as tmp:
            file=self.k.save(Path(tmp)/'knowledge.json')
            p=subprocess.run([sys.executable,str(script),'--knowledge',str(file),'--inspect'],cwd=tmp,capture_output=True,text=True,timeout=10)
            self.assertEqual(p.returncode,0,p.stderr)
            self.assertIn('simulation',p.stdout)
            self.assertEqual([p.name for p in Path(tmp).iterdir()],['knowledge.json'])
            before=file.read_bytes()
            p=subprocess.run([sys.executable,str(script),'--knowledge',str(file),'--log',str(file),'--inspect'],
                cwd=tmp,capture_output=True,text=True,timeout=10)
            self.assertEqual(p.returncode,2)
            self.assertIn('different files',p.stderr)
            self.assertEqual(file.read_bytes(),before)
            p=subprocess.run([sys.executable,str(script),'--knowledge',str(file),'--drive'],
                cwd=tmp,capture_output=True,text=True,timeout=10)
            self.assertEqual(p.returncode,2)
            self.assertIn('simulation knowledge cannot drive hardware',p.stderr)


if __name__=='__main__':unittest.main()
