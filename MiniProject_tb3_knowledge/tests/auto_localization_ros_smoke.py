"""Stationary fake robot + real AMCL; never use the physical robot's domain.

No initialpose message or motor publisher exists. The production setup monitor
must initialize AMCL globally, converge and capture the hidden simulated pose.
"""
import os
os.environ['ROS_DOMAIN_ID']='179'
os.environ.pop('ROS_LOCALHOST_ONLY',None)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'

import math
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.localization import start_localization
from core.maps import SlamMap
from core.model import Arena,Pose,wrap
from core.ros import SetupMonitor
from core.scan_matching import match_scan


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    import tf2_ros

    rclpy.init(args=[])
    node=rclpy.create_node('stationary_localization_test')
    session=monitor=matcher_patch=None
    use_hint='--hint' in sys.argv
    retry='--retry' in sys.argv
    match_calls=[]
    def transient_mismatch(*args,**kwargs):
        result=match_scan(*args,**kwargs)
        match_calls.append(result)
        return replace(result,reason='scan_map_mismatch') if len(match_calls)==1 else result
    with tempfile.TemporaryDirectory(prefix='auto_localization_test_') as temporary:
        try:
            folder=Path(temporary)
            states=np.zeros((60,60),dtype=np.uint8)
            states[[0,-1],:]=1;states[:,[0,-1]]=1
            if not use_hint:
                states[12:21,18:24]=1
                states[35:40,30:39]=1
            pixels=np.where(states==1,0,254).astype(np.uint8)
            image=folder/'arena.pgm'
            image.write_bytes(b'P5\n60 60\n255\n'+pixels.tobytes())
            source=folder/'arena.yaml'
            source.write_text(yaml.safe_dump(dict(image=str(image),resolution=.05,origin=[0.,0.,0.],
                negate=0,occupied_thresh=.65,free_thresh=.25)))
            slam=SlamMap.load(source)
            hidden=Pose(.55,.70,.4)
            assert slam.free(hidden.x,hidden.y,.153)
            laser=Pose(hidden.x+.02*math.cos(hidden.yaw),hidden.y+.02*math.sin(hidden.yaw),hidden.yaw)
            ranges=slam.scan(laser,count=360,max_range=8.)
            odom_pub=node.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
            scan_pub=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
            dynamic=tf2_ros.TransformBroadcaster(node)
            static=tf2_ros.StaticTransformBroadcaster(node)
            mount=TransformStamped()
            mount.header.frame_id='base_footprint';mount.child_frame_id='base_scan'
            mount.transform.translation.x=.02;mount.transform.rotation.w=1.
            static.sendTransform(mount)
            next_scan=0.

            def publish_sensors():
                nonlocal next_scan
                stamp=node.get_clock().now().to_msg()
                odom=Odometry();odom.header.stamp=stamp
                odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
                odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
                transform=TransformStamped();transform.header=odom.header
                transform.child_frame_id='base_footprint';transform.transform.rotation.w=1.
                dynamic.sendTransform(transform)
                if time.monotonic()<next_scan:return
                next_scan=time.monotonic()+.1
                scan=LaserScan()
                nanoseconds=node.get_clock().now().nanoseconds-110_000_000
                scan.header.stamp.sec,scan.header.stamp.nanosec=divmod(nanoseconds,1_000_000_000)
                scan.header.frame_id='base_scan'
                scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
                scan.angle_max=math.pi-scan.angle_increment
                scan.range_min=.16;scan.range_max=8.
                scan.scan_time=.1;scan.time_increment=.1/360
                scan.ranges=list(ranges);scan_pub.publish(scan)

            timer=node.create_timer(.02,publish_sensors)
            # Only suppress the viewer in this integration test; production
            # launch parameters, lifecycle and global-localization calls run.
            with patch('core.localization.LocalizationSession.ensure_rviz'):
                session=start_localization(source)
            if retry:
                matcher_patch=patch('core.scan_matching.match_scan',side_effect=transient_mismatch)
                matcher_patch.start()
            monitor=SetupMonitor(auto_localize=True,expected_map=slam,
                position_hint=(.68,.85) if use_hint else None,hint_radius=.35)
            monitor.start()
            until=time.monotonic()+55
            previous=None
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.02)
                snapshot=monitor.snapshot()
                if snapshot['reason']!=previous:
                    print('AUTO:',snapshot['reason'],snapshot.get('particle_support'),flush=True)
                    previous=snapshot['reason']
                assert not node.get_publishers_info_by_topic('/cmd_vel')
                assert not node.get_publishers_info_by_topic('/knowledge/cmd_vel')
                assert not node.get_publishers_info_by_topic('/initialpose')
                if snapshot['ready']:break
            assert snapshot['ready'],snapshot
            if retry:
                assert len(match_calls)>=2 and snapshot['match_attempts']>=2
                print('Transient scan mismatch recovered automatically; no retry click.',flush=True)
            captured,reference=monitor.capture(Arena(3.,3.))
            error=math.hypot(captured.x-hidden.x,captured.y-hidden.y)
            angle=abs(wrap(captured.yaw-hidden.yaw))
            assert error<.10 and angle<.15,(captured,hidden,error,angle)
            assert reference.mode=='localized' and reference.odom_source
            # Repeating search invalidates the earlier capture immediately.
            monitor.request_global_localization()
            assert not monitor.snapshot()['ready']
            print(f'Automatic localization passed ({"rough point in symmetric square" if use_hint else "global"}): position error {error:.3f}m, yaw error {math.degrees(angle):.2f}deg; no initialpose or motor topics.',flush=True)
        finally:
            if monitor:monitor.stop()
            if matcher_patch:matcher_patch.stop()
            if session:session.terminate()
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':main()
