"""Isolated fake ROS tests; never connect to the robot domain.

Run after sourcing ROS 2: python3 tb3_knowledge/tests/ros_smoke.py
Every sensor/motor topic is unique. Domain is set before importing rclpy.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import json
import io
from contextlib import redirect_stdout
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.model import Arena, Hardware, InitialReference, Knowledge, Pose
from core.ros import PoseBuffer, RuntimeState, SetupMonitor, _make_navigation_node, transform_pose, endpoint_id
from core.guard import make_guard_node, validate_velocity


class PureRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.reference = InitialReference('localized', 'test', 'odom', 'base_footprint', Pose(0, 0), time.time(), time.time(), odom_source='a'*48)
        self.knowledge = Knowledge(Arena(2, 2), Pose(.4, .4), (1.4, .4), reference=self.reference)

    def test_anchor_and_duplicate_age(self):
        state = RuntimeState(self.knowledge, 'test', True)
        stamp = self.reference.stamp+.01
        state.odometry(stamp, Pose(0, 0), 'odom', 'base_footprint', 0, 0, stamp, 1.)
        state.odometry(stamp, Pose(0, 0), 'odom', 'base_footprint', 0, 0, stamp+.1, 1.1)
        self.assertEqual(state.odom_seen, 1.)
        state.scan_seen = 1.4
        self.assertEqual(state.health(stamp+.51, 1.51), 'odometry_timeout')

    def test_wrong_robot_simulation_and_moved(self):
        with self.assertRaisesRegex(ValueError, 'wrong_robot'):
            RuntimeState(self.knowledge, 'another', True)
        with self.assertRaisesRegex(ValueError, 'simulation'):
            RuntimeState(Knowledge(Arena(2, 2), Pose(.4, .4), (1.4, .4)), 'tb3', True)
        state = RuntimeState(self.knowledge, 'test', True)
        stamp = self.reference.stamp+.1
        with self.assertRaisesRegex(ValueError, 'moved_since'):
            state.odometry(stamp, Pose(.04, 0), 'odom', 'base_footprint', 0, 0, stamp)

    def test_jump_and_frame(self):
        buffer = PoseBuffer()
        buffer.add(1., Pose(0, 0))
        with self.assertRaisesRegex(ValueError, 'jump'):
            buffer.add(1.05, Pose(.5, 0))
        state = RuntimeState(self.knowledge, 'test')
        with self.assertRaisesRegex(ValueError, 'frame'):
            state.odometry(2., Pose(0, 0), 'other', 'base_footprint', 0, 0, 2.)

    def test_source_identity_and_stationary_arm(self):
        from dataclasses import replace
        no_source = replace(self.knowledge, reference=replace(self.reference, odom_source=""))
        with self.assertRaisesRegex(ValueError, "missing_odometry_source"):
            RuntimeState(no_source, 'test', True)
        state = RuntimeState(self.knowledge, 'test', True)
        stamp = self.reference.stamp+.1
        state.odometry(stamp, Pose(0, 0), 'odom', 'base_footprint', 0, 0, stamp, 1.)
        state.odometry(stamp+.8, Pose(.03, 0), 'odom', 'base_footprint', 0, 0, stamp+.8, 1.8)
        self.assertFalse(state.ready_to_arm(1.8))
        self.assertEqual(state.stopped, 'robot_moved_since_capture')
        class Endpoint:
            endpoint_gid = bytes(range(16))
        self.assertEqual(len(endpoint_id(Endpoint())), 48)

    def test_transforms_and_caps(self):
        pose = transform_pose(Pose(1, 2, math.pi/2), Pose(0, 0), Pose(.2, 0))
        self.assertAlmostEqual(pose.x, 1.)
        self.assertAlmostEqual(pose.y, 2.2)
        self.assertEqual(validate_velocity(1., -4., max_speed=.08, max_angular=.8), (.08, -.8))
        with self.assertRaises(ValueError):
            validate_velocity(float('nan'), 0, max_speed=.08, max_angular=.8)


def run_ros_tests():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import tf2_ros
    rclpy.init(args=[])
    fake = rclpy.create_node('isolated_fake_hardware')
    prefix = '/knowledge_smoke_'+str(os.getpid())
    nodes, processes, orphan_guards = [fake], [], []
    monitor = None

    def pump(seconds, condition=None):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            for n in list(nodes):
                rclpy.spin_once(n, timeout_sec=.005)
            if condition is not None and condition():
                return True
        return condition() if condition is not None else True

    def stamp():
        return fake.get_clock().now().to_msg()

    try:
        # Both installed driver message conventions; caps, stale replay, service stop.
        for index, kind in enumerate(('stamped', 'twist')):
            wanted, output = prefix+f'/request{index}', prefix+f'/motor{index}'
            received = []
            sub = fake.create_subscription(TwistStamped if kind == 'stamped' else Twist, output, received.append, 10)
            pub = fake.create_publisher(TwistStamped, wanted, 10)
            guard = make_guard_node(input_topic=wanted, output_topic=output, output_type=kind,
                                    name='guard_smoke_'+str(index))
            nodes.append(guard)
            pump(.7)
            for _ in range(8):
                command = TwistStamped()
                command.header.stamp, command.header.frame_id = stamp(), 'base_footprint'
                command.twist.linear.x, command.twist.angular.z = 1., 3.
                pub.publish(command)
                pump(.04)
            def velocity(m):
                return m.twist.linear.x if kind == 'stamped' else m.linear.x
            assert any(0 < velocity(m) <= .080001 for m in received), kind+' did not forward'
            # Repeated stamp and no commands cannot refresh watchdog.
            pub.publish(command)
            pump(.45)
            assert velocity(received[-1]) == 0
            # Final zero followed by executor callbacks must never resurrect
            # the previous fresh command.
            command.header.stamp = stamp()
            pub.publish(command)
            assert pump(1, lambda: bool(received) and velocity(received[-1]) > 0)
            guard.cleanup()
            received.clear()
            pump(.15)
            assert guard.latched == 'shutdown'
            assert received and all(velocity(m) == 0 for m in received)
            client = fake.create_client(Trigger, '/guard_smoke_'+str(index)+'/stop')
            assert client.wait_for_service(timeout_sec=2)
            future = client.call_async(Trigger.Request())
            assert pump(2, future.done) and future.result().success
            assert guard.latched == 'operator_stop'
            nodes.remove(guard)
            guard.destroy_node()
            fake.destroy_publisher(pub)
            fake.destroy_subscription(sub)
            fake.destroy_client(client)

        # Conflicting teleop publisher latches a stop.
        wanted, output = prefix+'/conflict_request', prefix+'/conflict_motor'
        guard = make_guard_node(input_topic=wanted, output_topic=output, name='guard_conflict')
        nodes.append(guard)
        competitor = fake.create_publisher(TwistStamped, output, 10)
        assert pump(2, lambda: guard.latched == 'competing_motor_publisher')
        nodes.remove(guard)
        guard.destroy_node()
        fake.destroy_publisher(competitor)

        # A mismatched driver message type must fail closed.
        mismatch_sub = fake.create_subscription(Twist, prefix+'/wrongtype_motor', lambda m: None, 10)
        try:
            guard = make_guard_node(input_topic=prefix+'/wrongtype_request', output_topic=prefix+'/wrongtype_motor', name='guard_wrongtype')
        except Exception as exc:
            assert 'incompatible type' in str(exc), str(exc)
        else:
            nodes.append(guard)
            assert pump(2, lambda: guard.latched == 'motor_topic_type_mismatch')
            nodes.remove(guard)
            guard.destroy_node()
        fake.destroy_subscription(mismatch_sub)

        # Dry-run has diagnostics but no requested or motor command publisher.
        odom_topic, scan_topic = prefix+'/odom', prefix+'/scan'
        base, odom_frame, laser = 'smoke_base', 'smoke_odom', 'smoke_laser'
        reference = InitialReference('localized', 'test', odom_frame, base, Pose(0, 0), time.time(), time.time())
        knowledge = Knowledge(Arena(2, 2), Pose(.4, .4), (1.4, .4), reference=reference)
        dry = _make_navigation_node(knowledge, drive=False, scan_topic=scan_topic, odom_topic=odom_topic,
                command_topic=prefix+'/runtime_request', robot_id='test',ui_stream=True)
        nodes.append(dry)
        pump(.4)
        assert not fake.get_publishers_info_by_topic(prefix+'/runtime_request')
        assert not fake.get_publishers_info_by_topic(prefix+'/runtime_motor')
        dry.last_ui_update=0.
        stream=io.StringIO()
        with redirect_stdout(stream):dry.tick()
        packet=json.loads(next(line[len('TB3_DATA: '):] for line in stream.getvalue().splitlines() if line.startswith('TB3_DATA: ')))
        assert packet['drive'] is False and packet['requested']==[0.,0.]
        assert packet['points']==[] and packet['fingerprint']==knowledge.fingerprint
        # Reproduce the real startup failure: an old queued sweep arrives
        # first, then fresh scans. Old packets must never refresh freshness.
        def timed_scan(age):
            message=LaserScan();now=fake.get_clock().now().nanoseconds
            message.header.stamp.sec,message.header.stamp.nanosec=divmod(now-int(age*1e9),1_000_000_000)
            message.header.frame_id=laser
            message.angle_min=-math.pi;message.angle_increment=2*math.pi/360
            message.range_min=.16;message.range_max=8.;message.ranges=[1.8]*360
            return message
        dry.scan(timed_scan(.773))
        assert dry.state.stopped=='' and dry.latest_scan is None
        assert dry.state.scan_seen==0. and dry.discarded_old_scans==1
        fresh=timed_scan(.02);dry.scan(fresh)
        assert dry.latest_scan is fresh and dry.state.stopped==''
        accepted_at=dry.state.scan_seen
        dry.scan(timed_scan(.773))
        assert dry.latest_scan is fresh and dry.state.scan_seen==accepted_at
        assert dry.sensor_fault_info is None
        mismatch=Odometry()
        mismatch.header.stamp,mismatch.header.frame_id,mismatch.child_frame_id=stamp(),odom_frame,base
        mismatch.pose.pose.orientation.z=math.sin(.3/2)
        mismatch.pose.pose.orientation.w=math.cos(.3/2)
        dry.odom(mismatch,None)
        assert dry.state.stopped=='robot_moved_since_capture_or_odometry_reset'
        assert abs(dry.sensor_fault_info['reference_yaw_degrees']-math.degrees(.3))<1e-6
        assert dry.sensor_fault_info['reference_distance']==0.
        nodes.remove(dry)
        dry.cleanup()
        dry.destroy_node()

        future_probe=_make_navigation_node(knowledge,drive=False,scan_topic=scan_topic,
            odom_topic=odom_topic,command_topic=prefix+'/future_request',robot_id='test')
        nodes.append(future_probe)
        future_probe.scan(timed_scan(-.2))
        assert future_probe.state.stopped=='stale_or_future_scan'
        assert future_probe.sensor_fault_info['age']<-.06
        nodes.remove(future_probe);future_probe.cleanup();future_probe.destroy_node()

        # Run the public managed-process API with fake sensor/driver topics.
        motor = []
        motor_sub = fake.create_subscription(TwistStamped, prefix+'/runtime_motor', motor.append, 10)
        odom_pub = fake.create_publisher(Odometry, odom_topic, qos_profile_sensor_data)
        scan_pub = fake.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        static = tf2_ros.StaticTransformBroadcaster(fake)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp(), base, laser
        transform.transform.translation.x = .02
        transform.transform.rotation.w = 1.
        static.sendTransform(transform)
        scans_enabled = True
        clipped_scans = False
        old_scans = False
        def sensors():
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp(), odom_frame, base
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
            if not scans_enabled:
                return
            scan = LaserScan()
            # Every newest scan extends ahead of current odometry. A previous
            # complete sweep must remain available while new scans arrive.
            t = fake.get_clock().now().nanoseconds
            if old_scans:t-=800_000_000
            scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(t, 1_000_000_000)
            scan.header.frame_id = laser
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi-2*math.pi/360, 2*math.pi/360
            scan.range_min, scan.range_max, scan.time_increment = .16, 8., .00025
            scan.ranges = [1.8]*360
            if clipped_scans:scan.ranges=scan.ranges[:300]
            scan_pub.publish(scan)
        timer = fake.create_timer(.025, sensors)
        # Setup uses its own context/executor on a background thread. Its map
        # pose and synchronized raw odometry must survive a rotated arena.
        amcl_pub = fake.create_publisher(PoseWithCovarianceStamped, prefix+'/setup_pose', 10)
        def localization():
            amcl_pose = PoseWithCovarianceStamped()
            t = fake.get_clock().now().nanoseconds-100_000_000
            amcl_pose.header.stamp.sec, amcl_pose.header.stamp.nanosec = divmod(t, 1_000_000_000)
            amcl_pose.header.frame_id = 'map'
            amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y = .4, .4
            amcl_pose.pose.pose.orientation.w = 1.
            amcl_pose.pose.covariance[0] = amcl_pose.pose.covariance[7] = .001
            amcl_pose.pose.covariance[35] = .003
            amcl_pub.publish(amcl_pose)
        amcl_timer = fake.create_timer(.1, localization)
        monitor = SetupMonitor(odom_topic, prefix+'/setup_pose')
        monitor.start()
        assert pump(4, lambda: monitor.snapshot()['ready']), monitor.snapshot()
        captured, initial = monitor.capture(Arena(2, 2), 'test')
        assert abs(captured.x-.4) < 1e-6 and abs(captured.y-.4) < 1e-6
        assert initial.mode == 'localized' and initial.robot_id == 'test'
        assert initial.odom_frame == odom_frame and initial.base_frame == base
        assert initial.odom_source == endpoint_id(fake.get_publishers_info_by_topic(odom_topic)[0])
        # Live references bind the actual DDS publisher session, not its name.
        reference = initial
        knowledge = Knowledge(Arena(2, 2), Pose(.4, .4), (1.4, .4), reference=reference)
        monitor.stop()
        fake.destroy_timer(amcl_timer)
        fake.destroy_publisher(amcl_pub)
        # Direct shutdown regression: even a subsequently executed timer can
        # publish only zero after cleanup (no managed motor publisher here).
        requested = []
        request_sub = fake.create_subscription(TwistStamped, prefix+'/cleanup_request', requested.append, 10)
        probe = _make_navigation_node(knowledge, drive=True, scan_topic=prefix+'/unused_scan',
            odom_topic=odom_topic, command_topic=prefix+'/cleanup_request', robot_id='test')
        nodes.append(probe)
        pump(.3)
        probe.send(.05, 0.)
        assert pump(1, lambda: any(m.twist.linear.x > 0 for m in requested))
        probe.cleanup()
        assert probe.state.stopped == 'shutdown'
        requested.clear()
        pump(.2)
        assert requested and all(m.twist.linear.x == 0 for m in requested)
        nodes.remove(probe)
        probe.destroy_node()
        fake.destroy_subscription(request_sub)
        status = []
        status_sub = fake.create_subscription(String, '/knowledge/status', lambda m: status.append(json.loads(m.data)), 10)
        source = ('import sys;sys.path.insert(0,'+repr(str(Path(__file__).resolve().parents[1]))+');'
            'from core.model import Knowledge;from core.ros import run_navigation;'
            'k=Knowledge.from_document('+repr(knowledge.document())+');'
            'run_navigation(k,drive=True,robot_id="test",scan_topic='+repr(scan_topic)+',odom_topic='+repr(odom_topic)+','
            'command_topic='+repr(prefix+'/runtime_request')+',output_topic='+repr(prefix+'/runtime_motor')+')')
        # Reject a manually selected protocol mismatch BEFORE the guard
        # creates a motor publisher. Default auto detection is exercised below.
        wrong_type_source=source.replace('run_navigation(k,drive=True,','run_navigation(k,drive=True,output_type="twist",')
        wrong_type_process=subprocess.Popen([sys.executable,'-c',wrong_type_source],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        processes.append(wrong_type_process)
        assert pump(4,lambda:wrong_type_process.poll() is not None),'protocol preflight did not return'
        assert wrong_type_process.returncode!=0
        assert not fake.get_publishers_info_by_topic(prefix+'/runtime_motor')
        old_scans=True
        process = subprocess.Popen([sys.executable, '-c', source])
        processes.append(process)
        assert pump(5,lambda:any(s['discarded_old_scans']>0 for s in status)),status[-3:]
        assert all(m.twist.linear.x==0. and m.twist.angular.z==0. for m in motor)
        assert all(s['sensor_fault'] is None for s in status)
        old_scans=False
        assert pump(8, lambda: any(m.twist.linear.x > 0 for m in motor)), ('no runtime drive', status[-3:])
        assert any(s['requested'][0]>0 and s['scan_age']>=.08 for s in status),status[-3:]
        assert all(0 <= m.twist.linear.x <= .080001 and abs(m.twist.angular.z) <= .800001 for m in motor)
        # One clipped sweep must not latch malformed_scan. Sustained clipped
        # sweeps must not refresh the watchdog or permit indefinite driving.
        clipped_scans=True
        pump(.08)
        clipped_scans=False
        assert pump(1,lambda:status[-1]['reason']=='navigating'),status[-3:]
        assert all(s['reason']!='malformed_scan' for s in status)
        old_scans=True
        pump(.08)
        old_scans=False
        assert pump(1,lambda:status[-1]['reason']=='navigating'),status[-3:]
        assert any(s['discarded_old_scans']>0 for s in status)
        assert all(s['sensor_fault'] is None for s in status)
        # Full map localization activity is refused, even without map subscription.
        amcl = rclpy.create_node('amcl')
        nodes.append(amcl)
        assert pump(3, lambda: any(s['reason'] == 'full_map_localization_active' for s in status))
        assert pump(1, lambda: motor[-1].twist.linear.x == 0)
        nodes.remove(amcl)
        amcl.destroy_node()
        process.terminate()
        assert pump(3, lambda: process.poll() is not None)
        assert process.returncode == 0
        assert motor and motor[-1].twist.linear.x == 0 and motor[-1].twist.angular.z == 0
        # The child guard must be gone, not left publishing after normal cleanup.
        assert pump(3, lambda: not fake.get_publishers_info_by_topic(prefix+'/runtime_motor'))
        # A lost scan stream stops and remains stopped even if scans return.
        motor.clear()
        status.clear()
        process = subprocess.Popen([sys.executable, '-c', source])
        processes.append(process)
        assert pump(8, lambda: any(m.twist.linear.x > 0 for m in motor)), status[-3:]
        old_scans = True
        assert pump(2, lambda: any(s['reason'] == 'scan_timeout' for s in status))
        assert pump(1, lambda: motor[-1].twist.linear.x == 0)
        motor.clear()
        old_scans = False
        pump(.6)
        assert all(m.twist.linear.x == 0 for m in motor)
        process.terminate()
        assert pump(3, lambda: process.poll() is not None)
        assert pump(3, lambda: not fake.get_publishers_info_by_topic(prefix+'/runtime_motor'))

        # Replacing the driver at the same raw pose must invalidate a captured
        # handoff; pose and wall-clock timestamp alone cannot detect this reset.
        fake.destroy_publisher(odom_pub)
        odom_pub = fake.create_publisher(Odometry, odom_topic, qos_profile_sensor_data)
        assert pump(2, lambda: len(fake.get_publishers_info_by_topic(odom_topic)) == 1)
        changed_source = endpoint_id(fake.get_publishers_info_by_topic(odom_topic)[0])
        assert changed_source != knowledge.reference.odom_source
        motor.clear()
        status.clear()
        process = subprocess.Popen([sys.executable, '-c', source])
        processes.append(process)
        assert pump(4, lambda: any(s['reason'] == 'odometry_publisher_changed' for s in status)), status[-3:]
        pump(.2)
        assert all(m.twist.linear.x == 0 for m in motor)
        process.terminate()
        assert pump(3, lambda: process.poll() is not None)
        assert pump(3, lambda: not fake.get_publishers_info_by_topic(prefix+'/runtime_motor'))
        from dataclasses import replace
        new_knowledge = replace(knowledge, reference=replace(knowledge.reference,
            odom_source=changed_source, captured_at=time.time(), stamp=time.time()))
        source = source.replace(repr(knowledge.document()), repr(new_knowledge.document()))
        knowledge = new_knowledge

        # A killed controller cannot run cleanup; its separate guard still
        # expires the last command while notebook transport remains alive.
        motor.clear()
        status.clear()
        process = subprocess.Popen([sys.executable, '-c', source])
        processes.append(process)
        assert pump(8, lambda: any(m.twist.linear.x > 0 for m in motor)), status[-3:]
        child_file = Path('/proc')/str(process.pid)/'task'/str(process.pid)/'children'
        guard_pids = [int(value) for value in child_file.read_text().split()]
        assert guard_pids, 'managed guard process missing'
        orphan_guards.extend(guard_pids)
        process.kill()
        process.wait(timeout=2)
        motor.clear()
        assert pump(1, lambda: bool(motor) and motor[-1].twist.linear.x == 0)
        for guard_pid in guard_pids:
            os.kill(guard_pid, signal.SIGTERM)
        assert pump(3, lambda: not fake.get_publishers_info_by_topic(prefix+'/runtime_motor'))
        fake.destroy_timer(timer)
        fake.destroy_subscription(motor_sub)
        fake.destroy_subscription(status_sub)
        fake.destroy_publisher(scan_pub)
        fake.destroy_publisher(odom_pub)

        print('ROS smoke passed: both Twist types, caps, timeout/replay, stop, conflict/wrong-type refusal, dry-run, setup capture, live sensors, source reset rejection, shutdown latch, scan-loss latch, controller crash timeout, managed guard cleanup, full-map refusal.')
    finally:
        if monitor is not None:
            monitor.stop()
        for guard_pid in orphan_guards:
            try:
                os.kill(guard_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for node in reversed(nodes):
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(PureRuntimeTests)
    result = unittest.TextTestRunner().run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    run_ros_tests()
