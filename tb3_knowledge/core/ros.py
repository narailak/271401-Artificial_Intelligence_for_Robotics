"""Live ROS adapters, loaded only when setup/navigation is requested.

Neither navigation nor guard loads a map or calls a Nav2 action. Setup alone
observes AMCL; navigation anchors the field to a captured odometry reference.
"""
from __future__ import annotations

from collections import deque
from bisect import bisect_right
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from .model import InitialReference, Pose, wrap
from .navigation import MIN_SCAN_COVERAGE_PERCENT


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def endpoint_id(info):
    """Opaque DDS publisher identity, not obstacle data or authentication."""
    if isinstance(info, dict):
        value = info.get("publisher_gid", ())
    else:
        value = getattr(info, "endpoint_gid", getattr(info, "publisher_gid", ()))
    raw = bytes(value)
    # Jazzy/FastDDS exposes 16-byte GUIDs; some RMWs expose rmw_gid_t's
    # 24-byte storage. Normalize the unused storage for one schema length.
    return raw.ljust(24, b"\0").hex() if raw and len(raw) <= 24 else ""


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values):
        raise ValueError("nonfinite orientation")
    norm = sum(x*x for x in values)
    if abs(norm - 1) > .02:
        raise ValueError("invalid orientation norm")
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
    if abs(roll) > .12 or abs(pitch) > .12:
        raise ValueError("nonplanar orientation")
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def compose(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose(a.x+c*b.x-s*b.y, a.y+s*b.x+c*b.y, wrap(a.yaw+b.yaw))


def inverse(a):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose(-c*a.x-s*a.y, s*a.x-c*a.y, -a.yaw)


def transform_pose(start, reference, current):
    return compose(start, compose(inverse(reference), current))


class PoseBuffer:
    """Timestamp-ordered planar odometry; duplicate stamps never refresh age."""
    def __init__(self):
        self.samples = deque(maxlen=1200)

    def add(self, stamp, pose):
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("invalid_odometry_stamp")
        if self.samples:
            old_stamp, old_pose = self.samples[-1]
            if stamp == old_stamp:
                return False
            if stamp < old_stamp:
                raise ValueError("odometry_time_reset")
            dt = stamp-old_stamp
            if (math.hypot(pose.x-old_pose.x, pose.y-old_pose.y) > .04+.5*dt
                    or abs(wrap(pose.yaw-old_pose.yaw)) > .10+3.0*dt):
                raise ValueError("odometry_jump")
        self.samples.append((stamp, pose))
        while len(self.samples) > 2 and stamp-self.samples[0][0] > 10:
            self.samples.popleft()
        return True

    def at(self, stamp, tolerance=.06):
        if not self.samples:
            raise ValueError("waiting_odometry")
        if stamp < self.samples[0][0]:
            if self.samples[0][0]-stamp <= tolerance:
                return self.samples[0][1]
            raise ValueError("odometry_history_missing")
        if stamp > self.samples[-1][0]:
            if stamp-self.samples[-1][0] <= tolerance:
                return self.samples[-1][1]
            raise ValueError("waiting_scan_odometry")
        for (ta, a), (tb, b) in zip(self.samples, list(self.samples)[1:]):
            if ta <= stamp <= tb:
                fraction = (stamp-ta)/(tb-ta)
                return Pose(a.x+(b.x-a.x)*fraction, a.y+(b.y-a.y)*fraction,
                            wrap(a.yaw+wrap(b.yaw-a.yaw)*fraction))
        return self.samples[-1][1]

    def at_many(self, stamps, tolerance=0):
        """Interpolate one sweep against one snapshot, without extrapolation."""
        samples=list(self.samples)
        if not samples:raise ValueError('waiting_odometry')
        times=[item[0] for item in samples]
        result=[]
        for stamp in stamps:
            if stamp<times[0]:
                if times[0]-stamp>tolerance:raise ValueError('odometry_history_missing')
                result.append(samples[0][1]);continue
            if stamp>times[-1]:
                if stamp-times[-1]>tolerance:raise ValueError('waiting_scan_odometry')
                result.append(samples[-1][1]);continue
            index=bisect_right(times,stamp)
            if index==len(samples):result.append(samples[-1][1]);continue
            ta,a=samples[index-1];tb,b=samples[index]
            fraction=(stamp-ta)/(tb-ta)
            result.append(Pose(a.x+(b.x-a.x)*fraction,a.y+(b.y-a.y)*fraction,
                               wrap(a.yaw+wrap(b.yaw-a.yaw)*fraction)))
        return result


def complete_scan(scans, poses, now, timeout):
    """Newest fresh sweep whose every beam has recorded odometry coverage."""
    if not scans:raise ValueError('waiting_scan')
    if not poses.samples:raise ValueError('waiting_odometry')
    first,last=poses.samples[0][0],poses.samples[-1][0]
    recent=covered_start=False
    for scan in reversed(scans):
        start=stamp_seconds(scan.header.stamp)
        if not -.06<=now-start<=timeout:continue
        recent=True
        if start<first:continue
        covered_start=True
        end=start+(len(scan.ranges)-1)*scan.time_increment
        if end<=last:return scan
    if not recent:raise ValueError('stale_or_future_scan')
    if not covered_start:raise ValueError('odometry_history_missing')
    raise ValueError('waiting_scan_odometry')


def scan_coverage(scan):
    """Latest raw scan completeness, not a probability of collision-free motion."""
    total=len(scan.ranges) if scan is not None else 0
    valid=sum(math.isfinite(value) and scan.range_min<=value<=scan.range_max for value in scan.ranges) if total else 0
    return dict(scan_total_beams=total,scan_valid_beams=valid,scan_missing_beams=total-valid,
        scan_coverage_percent=100*valid/total if total else None,
        scan_missing_percent=100*(total-valid)/total if total else None)


def aligned_scan_observation(scan, scans, poses, field_pose, speed, mount, now, hardware, *, angular_speed=0.):
    """Deskew live rays, interpolate narrow NaN gaps, then consult live history.

    No saved obstacle data enters here. Both each ray's sensor origin and its
    endpoint are transformed using odometry recorded at that beam's timestamp.
    Cached evidence keeps its original age for the navigator's braking budget.
    """
    from .lidar_history import Ray, Sweep, fill_missing
    from .navigation import Observation
    from .clearance import UnknownSector
    from .scan_filter import bridge_short_dropouts
    stamp=stamp_seconds(scan.header.stamp)
    current_inverse=inverse(poses.samples[-1][1])

    def rays_for(sweep, filtered=None):
        start=stamp_seconds(sweep.header.stamp)
        stamps=[start+i*sweep.time_increment for i in range(len(sweep.ranges))]
        beam_poses=poses.at_many(stamps)
        rays,origins,angles=[],[],[]
        for index,(value,pose,beam_stamp) in enumerate(zip(sweep.ranges if filtered is None else filtered,beam_poses,stamps)):
            sensor=compose(compose(current_inverse,pose),Pose(*mount))
            angle=wrap(sweep.angle_min+index*sweep.angle_increment+sensor.yaw)
            origin=(sensor.x,sensor.y)
            origins.append(origin);angles.append(angle)
            if not math.isfinite(value) or not sweep.range_min<=value<=sweep.range_max:
                rays.append(None);continue
            length=min(float(value),hardware.scan_range)
            rays.append(Ray(origin,(sensor.x+length*math.cos(angle),sensor.y+length*math.sin(angle)),beam_stamp))
        pairs=list(zip(angles,angles[1:]))
        spacing=max([abs(sweep.angle_increment)]+[abs(wrap(b-a)) for a,b in pairs])
        return Sweep(tuple(rays),sweep.angle_increment),origins,angles,spacing

    filtered,bridged,bridge_spacing=bridge_short_dropouts(scan.ranges,scan.angle_increment,scan.range_min,scan.range_max)
    current,origins,angles,spacing=rays_for(scan,filtered)
    if bridged:
        # Deskew can widen the original angular interval while the robot turns.
        spacing=max(spacing,bridge_spacing*spacing/abs(scan.angle_increment))
    points=[ray.endpoint for ray in current.rays if ray is not None]
    # Remaining NaN dropouts can use history. Out-of-range finite/negative/Inf
    # readings stay unknown, and finite close hits always survive unchanged.
    missing=[(origins[i],angles[i],spacing) for i,value in enumerate(scan.ranges) if math.isnan(value) and i not in bridged]
    invalid=[angles[i] for i,ray in enumerate(current.rays) if ray is None and not math.isnan(scan.ranges[i])]
    # Clockwise motion or clipped first/last returns can leave an unseen seam.
    # Represent that particular wedge instead of inflating every return by it.
    sign=1 if scan.angle_increment>0 else -1
    span=sum(sign*wrap(b-a) for a,b in zip(angles,angles[1:]))
    gap=2*math.pi-span
    seam=None
    if gap>abs(scan.angle_increment)+1e-5:
        center=wrap(angles[0]-sign*gap/2)
        # Bound every possible displaced origin by a disk about the current
        # mount. Shrink the inner radius and widen the angle by that disk;
        # unobserved sectors never gain fictitious near-field clearance.
        displacement=max(math.dist(origin,mount[:2]) for origin in origins)
        widen=math.asin(min(1.,displacement/hardware.lidar_min))
        seam=UnknownSector(mount[:2],center,gap/2+widen,max(0.,hardware.lidar_min-displacement))
    history=[]
    max_age=min(.50,hardware.scan_timeout)
    if missing or seam is not None:
        first,last=poses.samples[0][0],poses.samples[-1][0]
        candidates=[]
        for previous in scans:
            start=stamp_seconds(previous.header.stamp)
            end=start+(len(previous.ranges)-1)*previous.time_increment
            if (previous.header.frame_id==scan.header.frame_id and first<=start<stamp
                    and end<=last and 0<=now-start<=max_age):
                candidates.append(previous)
        for previous in candidates[-3:]:
            sweep,_,_,_=rays_for(previous)
            history.append(sweep)
        points,unresolved,oldest=fill_missing(points,missing,history,now,max_age,
            hardware.scan_range,minimum_range=hardware.lidar_min)
        invalid.extend(unresolved)
        if oldest is not None:
            stamp=min(stamp,oldest)
            # Historical fans admit at most twice their nominal beam spacing.
            # Increase inflation only; raw scan coverage is validated separately.
            spacing=max([spacing]+[2*abs(sweep.angle_increment) for sweep in history])
    if seam is not None and seam.minimum>0 and 2*seam.half_width<=.2:
        points,unresolved,oldest=fill_missing(points,[(seam.origin,seam.angle,2*seam.half_width)],
            history,now,max_age,hardware.scan_range,minimum_range=seam.minimum)
        if not unresolved:seam=None
        if oldest is not None:
            stamp=min(stamp,oldest)
            spacing=max([spacing]+[2*abs(sweep.angle_increment) for sweep in history])
    by_angle=dict(zip(angles,origins))
    unknown=[UnknownSector(by_angle[angle],angle,spacing/2,hardware.lidar_min) for angle in invalid]
    if seam is not None:
        unknown.append(seam)
        invalid.append(seam.angle)
    return Observation(pose=field_pose,ranges=tuple(scan.ranges),angle_min=scan.angle_min,
        angle_increment=scan.angle_increment,stamp=stamp,mount=mount,linear_speed=speed,
        points=tuple(points),invalid_angles=tuple(invalid),beam_spacing=spacing,unknown_sectors=tuple(unknown),angular_speed=angular_speed,
        scan_coverage_percent=scan_coverage(scan)['scan_coverage_percent'],interpolated_beams=len(bridged))


def runtime_status_text(reason,drive):
    messages={
        'full_map_localization_active':'หยุด: AMCL/SLAM ยังเปิดอยู่ ปิด Localization จาก Program 1 แล้วหยุดและเริ่ม Program 2 ใหม่',
        'waiting_odometry':'กำลังรอข้อมูล odometry จากหุ่น',
        'waiting_scan':'กำลังรอข้อมูล LiDAR จากหุ่น',
        'waiting_scan_odometry':'กำลังรอ odometry ให้ครอบคลุม LiDAR ทั้งรอบ',
        'odometry_history_missing':'กำลังเก็บประวัติ odometry สำหรับชดเชยเวลาสแกน',
        'waiting_lidar_TF':'กำลังรอ TF ระหว่างตัวหุ่นกับ LiDAR',
        'keep_robot_stationary':'ให้หุ่นอยู่นิ่งระหว่างตรวจ Start',
        'waiting_guard_and_graph':'กำลังตรวจ guard และการเชื่อมต่อ ROS ก่อนเริ่มเดิน',
        'odometry_publisher_changed':'หยุด: แหล่ง odometry เปลี่ยน กลับไปรับ Start และส่งออก knowledge ใหม่',
        'robot_moved_since_capture_or_odometry_reset':'หยุด: odometry ไม่ตรงกับ reference ที่ส่งออก กลับ Program 1 รับ Start และส่งออกใหม่',
        'robot_moved_since_capture':'หยุด: หุ่นเปลี่ยนตำแหน่งหลังรับ Start กลับไปรับ Start ใหม่',
        'scan_timeout':'หยุดรอ: ข้อมูล LiDAR เก่าเกินกำหนด',
        'odometry_timeout':'หยุดรอ: ข้อมูล odometry เก่าเกินกำหนด',
        'stale_or_future_odometry':'หยุด: เวลา odometry เก่าหรือเร็วกว่าเวลาคอม ตรวจการ sync เวลาของคอมและ Pi แล้วเริ่มใหม่',
        'stale_or_future_scan':'หยุด: เวลา LiDAR เก่าหรือเร็วกว่าเวลาคอม ตรวจการ sync เวลาของคอมและ Pi แล้วเริ่มใหม่',
        'no_feasible_gap':'หยุดรอ: ยังไม่พบช่องทางที่มีระยะเผื่อพอ ตรวจสิ่งกีดขวางและมุม LiDAR ที่ขาดข้อมูล',
        'unknown_rotation_clearance':'หยุดรอ: LiDAR ยังยืนยันพื้นที่รอบตัวสำหรับหมุนไม่ได้ ตรวจสิ่งบังและช่องสแกนที่ขาด',
        'rotation_drift_clearance':'หยุดรอ: ระยะหยุดไหลก่อนหมุนยังใกล้จุดวัดจริงหรือขอบสนามเกินไป',
        'footprint_clearance':'หยุดรอ: ข้อมูล LiDAR พบจุดอยู่ในวงหุ่นรวมระยะเผื่อ',
        'observing_for_gap':'กำลังหมุนช้า ๆ เพื่อตรวจช่องทางใหม่ โดยตรวจวงหุ่นกับ LiDAR ก่อนหมุน',
        'advancing_for_scan':'ขยับหน้าช้า ๆ เพื่อเก็บ LiDAR เพิ่ม เฉพาะทางที่ตรวจแล้ว ไม่เกิน 5 cm / 0.02 m/s',
        'waiting_scan_after_advance':'หยุดเก็บ LiDAR หลังขยับตรวจครบระยะหรือเวลา รอข้อมูลยืนยันก่อนเดินต่อ',
        'waiting_scan_clearance':'หยุดรอ: LiDAR หรือระยะเบรกยังไม่ยืนยันว่าขยับหน้าต่อได้',
        'insufficient_scan_coverage':f'หยุดรอ: สแกนที่ใช้ตรวจทางอ่านได้ไม่ถึง {MIN_SCAN_COVERAGE_PERCENT:.0f}%',
        'incomplete_scan':'รอสแกนรอบถัดไป: รอบนี้ข้อมูลมุมยังไม่ครบ',
        'malformed_scan':'หยุด: ข้อมูล LiDAR ผิดรูปแบบ ตรวจสถานะเซนเซอร์แล้วเริ่มใหม่',
        'control_loop_overrun':'หยุด: คอมคำนวณไม่ทันช่วงเวลาที่อนุญาต หยุดและเริ่มการทดลองใหม่',
        'guard_motor_topic_type_mismatch':'หยุด: ชนิด /cmd_vel ไม่ตรงกับหุ่น เลือก auto หรือชนิดที่ driver รับ แล้วเริ่มใหม่',
        'guard_competing_motor_publisher':'หยุด: มีโปรแกรมอื่นส่ง /cmd_vel อยู่ ปิด teleop/ตัวนำทางชุดอื่น แล้วเริ่มใหม่',
        'guard_waiting_motor_subscriber':'รอ driver หุ่นเชื่อมต่อ /cmd_vel',
        'waiting_motor_subscriber':'รอ driver หุ่นเชื่อมต่อ /cmd_vel ตรวจ bringup และ ROS_DOMAIN_ID',
        'arrived_estimate':'หยุด: ตำแหน่งประมาณถึง Goal แล้ว',
        'operator_stop':'หยุดตามคำสั่งผู้ใช้',
    }
    return messages.get(reason,('ตัวนำทาง' if drive else 'ตรวจเซนเซอร์ ไม่สั่งมอเตอร์')+': '+reason)


class SetupMonitor:
    """Background AMCL/odometry monitor. It never publishes motor commands.

    snapshot() is safe on the Tk thread. capture() propagates the map pose at
    AMCL's timestamp to the latest odometry sample, then stores both together.
    AMCL must already be running in a separate setup-only ROS launch.
    auto_localize adds stationary global initialization and scan/map validation;
    it calls localization services only and creates no motor publisher.
    """
    def __init__(self, odom_topic="/odom", localization_topic="/amcl_pose", *, auto_localize=False, expected_map=None, hardware=None,
                 position_hint=None,hint_radius=.35):
        self.odom_topic, self.localization_topic = odom_topic, localization_topic
        self._lock = threading.RLock()
        self._quit = threading.Event()
        self._thread = None
        self._error = ""
        self._node = None
        self._context = None
        self._buffer = PoseBuffer()
        self._localization = None
        self._odom_seen = 0.
        self._stationary_since = 0.
        self._odom_frame, self._base_frame = "odom", "base_footprint"
        self._odom_source = ""
        self.auto_localize=auto_localize
        self.expected_map=expected_map
        from .model import Hardware
        self.hardware=hardware or Hardware()
        self._global_request=threading.Event()
        self._global_status='waiting_global_localization' if auto_localize else ''
        self._global_epoch=math.inf if auto_localize else 0.
        self._search_started=0.
        self._map_ready=False
        self._map_error=''
        self._scan_stamp=0.
        self._particles_state=None
        self._pose_history=deque(maxlen=100)
        self._latest_setup_scan=None
        self._matching_status='waiting_scan_match' if auto_localize and expected_map is not None else ''
        self._match_generation=0
        self._match_attempts=0
        self._scan_agreement=None
        self._scan_quality=None
        self._matched_pose=None
        self.position_hint,self.hint_radius=self._validated_hint(position_hint,hint_radius)
        if auto_localize:self._global_request.set()

    @staticmethod
    def _validated_hint(position_hint,hint_radius):
        if not math.isfinite(hint_radius) or not .10<=hint_radius<=2.:
            raise ValueError('ระยะค้นหารอบจุดคร่าว ๆ ต้องอยู่ระหว่าง 0.10–2.00 เมตร')
        if position_hint is not None:
            if len(position_hint)!=2 or not all(math.isfinite(v) for v in position_hint):
                raise ValueError('ตำแหน่งคร่าว ๆ ต้องเป็นพิกัด x/y ที่เป็นตัวเลขจำกัด')
            position_hint=tuple(float(v) for v in position_hint)
        return position_hint,float(hint_radius)

    def set_search_hint(self,position_hint,hint_radius=.35):
        """A changed positional prior invalidates any in-flight search result."""
        if not self.auto_localize:raise ValueError('automatic_localization_disabled')
        hint,distance=self._validated_hint(position_hint,hint_radius)
        with self._lock:
            if (hint,distance)==(self.position_hint,self.hint_radius):return False
            self.position_hint,self.hint_radius=hint,distance
            self.request_global_localization()
            return True

    def request_global_localization(self):
        """Queue a fresh global search; the ROS worker owns service calls."""
        if not self.auto_localize:raise ValueError('automatic_localization_disabled')
        with self._lock:
            self._global_epoch=math.inf
            self._global_status='waiting_global_localization'
            self._localization=self._particles_state=None
            self._pose_history.clear()
            self._search_started=0.
            self._matching_status='waiting_scan_match' if self.expected_map is not None else ''
            self._match_generation+=1
            self._match_attempts=0
            self._scan_agreement=None
            self._scan_quality=None
            self._matched_pose=None
            self._global_request.set()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._quit.clear()
        self._thread = threading.Thread(target=self._worker, name="knowledge-setup", daemon=True)
        self._thread.start()

    def _worker(self):
        matcher_pool=None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from std_srvs.srv import Empty
            self._context = Context()
            rclpy.init(args=[], context=self._context)
            self._node = Node("knowledge_setup", context=self._context)
            self._node.create_subscription(Odometry, self.odom_topic, self._odom, qos_profile_sensor_data)
            self._node.create_subscription(PoseWithCovarianceStamped, self.localization_topic, self._amcl, 10)
            refresh = self._node.create_client(Empty, "/request_nomotion_update")
            pending = None
            next_refresh = 0.
            global_pending=state_pending=None
            global_client=state_client=None
            active_since=next_state=0.
            matcher_future=initial_pending=None
            pending_seed=None
            matcher_generation=-1
            matcher_odom=None
            next_match=0.
            last_match_scan=None
            if self.auto_localize:
                from nav_msgs.msg import OccupancyGrid
                from nav2_msgs.msg import ParticleCloud
                from sensor_msgs.msg import LaserScan
                from lifecycle_msgs.srv import GetState
                from rclpy.qos import QoSProfile,DurabilityPolicy
                map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self._node.create_subscription(OccupancyGrid,'/map',self._map,map_qos)
                self._node.create_subscription(LaserScan,'/scan',self._setup_scan,qos_profile_sensor_data)
                self._node.create_subscription(ParticleCloud,'/particle_cloud',self._particles,qos_profile_sensor_data)
                global_client=self._node.create_client(Empty,'/reinitialize_global_localization')
                state_client=self._node.create_client(GetState,'/amcl/get_state')
                if self.expected_map is not None:
                    from concurrent.futures import ThreadPoolExecutor
                    import tf2_ros
                    from rclpy.time import Time
                    from nav2_msgs.srv import SetInitialPose
                    matcher_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='setup-scan-match')
                    tf_buffer=tf2_ros.Buffer()
                    tf_listener=tf2_ros.TransformListener(tf_buffer,self._node)
                    initial_client=self._node.create_client(SetInitialPose,'/set_initial_pose')
                    self._node.create_subscription(PoseWithCovarianceStamped,'/initialpose',self._manual_initial_pose,10)
            executor = SingleThreadedExecutor(context=self._context)
            executor.add_node(self._node)
            while not self._quit.is_set() and rclpy.ok(context=self._context):
                executor.spin_once(timeout_sec=.05)
                now=time.monotonic()
                if self.auto_localize and self._global_request.is_set():
                    if global_pending is not None and global_pending.done():
                        global_pending.result()
                        with self._lock:
                            self._global_epoch=self._node.get_clock().now().nanoseconds*1e-9
                            self._global_status=''
                            self._localization=self._particles_state=None
                            self._pose_history.clear()
                            self._search_started=now
                            self._global_request.clear()
                        global_pending=None
                    if state_pending is not None and state_pending.done():
                        active=state_pending.result().current_state.id==3
                        active_since=(active_since or now) if active else 0.
                        state_pending=None
                    if now>=next_state and state_pending is None and state_client.service_is_ready():
                        state_pending=state_client.call_async(GetState.Request());next_state=now+.5
                    with self._lock:
                        snapshot=self.snapshot()
                        can_search=(not snapshot['error'] and self._map_ready and snapshot['stationary'] and snapshot['odom_age']<=.4
                            and -.06<=snapshot['odom_age'] and -.06<=self._node.get_clock().now().nanoseconds*1e-9-self._scan_stamp<=.6)
                    # AMCL's global service requires an active filter with a
                    # received map. Let map/lifecycle discovery settle first.
                    if (self._global_request.is_set() and global_pending is None and can_search
                            and active_since and now-active_since>=1. and global_client.service_is_ready()):
                        global_pending=global_client.call_async(Empty.Request())
                        with self._lock:self._global_status='initializing_global_localization'
                if matcher_pool is not None and not self._global_request.is_set():
                    from .scan_matching import match_scan
                    if matcher_future is not None and matcher_future.done():
                        try:result=matcher_future.result()
                        except ValueError as exc:
                            result=None
                            if matcher_generation==self._match_generation:
                                with self._lock:self._matching_status=str(exc)
                        if result is not None and matcher_generation==self._match_generation:
                            with self._lock:
                                self._scan_agreement=result.agreement
                                self._scan_quality=dict(separation=result.separation,trimmed_error=result.trimmed_error,endpoint_score=result.endpoint_score)
                                self._matching_status=result.reason if result.reason!='matched' else 'waiting_initial_pose_service'
                            if result.reason=='matched':pending_seed=result.pose
                        matcher_future=None
                        next_match=now+1.
                    if initial_pending is not None and initial_pending.done():
                        initial_pending.result();initial_pending=None
                        if matcher_generation==self._match_generation:
                            with self._lock:
                                self._global_epoch=self._node.get_clock().now().nanoseconds*1e-9
                                self._matching_status=''
                                self._localization=self._particles_state=None
                                self._pose_history.clear()
                    if (matcher_future is None and self._matching_status in ('waiting_scan_match','waiting_lidar_TF',
                            'scan_map_mismatch','localization_ambiguous','insufficient_localization_scan',
                            'no_localization_match','robot_moved_during_search')
                            and now>=next_match and now-self._search_started<45.
                            and self._latest_setup_scan is not None
                            and last_match_scan!=(self._match_generation,self._scan_stamp)
                            and self.snapshot()['stationary']):
                        scan=self._latest_setup_scan
                        try:
                            transform=tf_buffer.lookup_transform(self._base_frame,scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
                            mount=(transform.translation.x,transform.translation.y,yaw_from_quaternion(transform.rotation))
                        except Exception:
                            with self._lock:self._matching_status='waiting_lidar_TF'
                        else:
                            if (not all(math.isfinite(v) for v in mount) or math.hypot(*mount[:2])>.35
                                    or abs(transform.translation.z)>.5):raise ValueError('invalid_lidar_mount')
                            with self._lock:
                                # Freeze scope and generation together: a UI
                                # hint edit must never revive an older search.
                                if self._global_request.is_set():continue
                                matcher_odom=self._buffer.samples[-1][1]
                                matcher_generation=self._match_generation
                                position_hint,hint_radius=self.position_hint,self.hint_radius
                                self._matching_status='matching_scan_to_map'
                                self._match_attempts+=1
                                last_match_scan=(matcher_generation,self._scan_stamp)
                            matcher_future=matcher_pool.submit(match_scan,self.expected_map,tuple(scan.ranges),scan.angle_min,
                                scan.angle_increment,mount,radius=self.hardware.safe_radius,
                                minimum=self.hardware.lidar_min,maximum=self.hardware.scan_range,
                                position_hint=position_hint,hint_radius=hint_radius,cancel=self._quit)
                    if pending_seed is not None and matcher_generation!=self._match_generation:pending_seed=None
                    if (pending_seed is not None and initial_pending is None and initial_client.service_is_ready()
                            and self._map_ready and self.snapshot()['stationary'] and -.06<=self.snapshot()['odom_age']<=.4):
                        current=self._buffer.samples[-1][1]
                        if math.hypot(current.x-matcher_odom.x,current.y-matcher_odom.y)>.01 or abs(wrap(current.yaw-matcher_odom.yaw))>.025:
                            with self._lock:self._matching_status='robot_moved_during_search'
                        else:
                            seed=transform_pose(pending_seed,matcher_odom,current)
                            request=SetInitialPose.Request()
                            request.pose.header.frame_id='map'
                            request.pose.header.stamp=self._node.get_clock().now().to_msg()
                            request.pose.pose.pose.position.x=seed.x;request.pose.pose.pose.position.y=seed.y
                            request.pose.pose.pose.orientation.z=math.sin(seed.yaw/2)
                            request.pose.pose.pose.orientation.w=math.cos(seed.yaw/2)
                            request.pose.pose.covariance[0]=request.pose.pose.covariance[7]=.0025
                            request.pose.pose.covariance[35]=.003
                            initial_pending=initial_client.call_async(request)
                            with self._lock:
                                self._matching_status='initializing_scan_match'
                                self._matched_pose=seed
                        pending_seed=None
                if (not self._global_request.is_set() and now >= next_refresh and refresh.service_is_ready()
                        and (pending is None or pending.done())):
                    pending = refresh.call_async(Empty.Request())
                    next_refresh = now+(.5 if self.auto_localize else 1.)
            executor.shutdown(timeout_sec=1.)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
        finally:
            if matcher_pool is not None:matcher_pool.shutdown(wait=False,cancel_futures=True)
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            if self._context is not None and self._context.ok():
                self._context.shutdown()

    def _map(self,msg):
        valid=(msg.header.frame_id.lstrip('/')=='map' and msg.info.width>0 and msg.info.height>0
            and math.isfinite(msg.info.resolution) and msg.info.resolution>0
            and len(msg.data)==msg.info.width*msg.info.height and any(value==0 for value in msg.data))
        if valid and self.expected_map is not None:
            import numpy as np
            slam=self.expected_map
            origin=msg.info.origin
            try:yaw=yaw_from_quaternion(origin.orientation)
            except ValueError:valid=False
            if valid:
                valid=(msg.info.width==slam.width and msg.info.height==slam.height
                    and abs(msg.info.resolution-slam.resolution)<1e-6
                    and math.hypot(origin.position.x-slam.origin[0],origin.position.y-slam.origin[1])<1e-6
                    and abs(wrap(yaw-slam.origin[2]))<1e-6)
                if valid:
                    expected=np.array([0,100,-1],dtype=np.int16)[slam.states[::-1]].ravel()
                    valid=bool(np.array_equal(np.asarray(msg.data),expected))
        with self._lock:
            self._map_ready=valid
            self._map_error='' if valid else 'localization_map_mismatch'

    def _setup_scan(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        # NaN rays do not participate in AMCL. Require enough actual returns
        # spread around the robot, rather than a single visible wall segment.
        now=self._node.get_clock().now().nanoseconds*1e-9 if self._node is not None else time.time()
        if (not -.06<=now-stamp<=.6 or not msg.header.frame_id
                or not all(math.isfinite(v) for v in (msg.angle_increment,msg.angle_min,msg.range_min,msg.range_max))
                or msg.angle_increment==0 or msg.range_min<0 or msg.range_max<=msg.range_min):return
        sectors=set()
        valid=0
        for index,value in enumerate(msg.ranges):
            if math.isfinite(value) and max(self.hardware.lidar_min,msg.range_min)<=value<=min(self.hardware.scan_range,msg.range_max):
                valid+=1
                sectors.add(int(((msg.angle_min+index*msg.angle_increment)%(2*math.pi))/(2*math.pi/12)))
        with self._lock:
            if valid>=30 and len(sectors)>=6 and stamp>self._scan_stamp:
                self._scan_stamp=stamp;self._latest_setup_scan=msg

    def _manual_initial_pose(self,msg):
        """Explicit RViz assistance remains available for a symmetric scene."""
        try:
            pose=Pose(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw_from_quaternion(msg.pose.pose.orientation))
            now=self._node.get_clock().now().nanoseconds*1e-9
            if (msg.header.frame_id.lstrip('/')!='map' or not -.06<=now-stamp_seconds(msg.header.stamp)<=1.
                    or not self.expected_map.free(pose.x,pose.y,self.hardware.safe_radius)):return
            with self._lock:
                if self._global_request.is_set() or self._matching_status=='initializing_scan_match':return
                self._match_generation+=1
                self._matching_status=''
                self._global_epoch=now
                self._localization=self._particles_state=None
                self._pose_history.clear()
                self._matched_pose=pose
                self._scan_agreement=None
                self._scan_quality=None
        except (ValueError,TypeError):return

    def _particles(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':return
        try:
            particles=tuple((Pose(p.pose.position.x,p.pose.position.y,yaw_from_quaternion(p.pose.orientation)),float(p.weight))
                for p in msg.particles)
            if not particles or any(not math.isfinite(w) or w<0 for _,w in particles):return
            stamp=stamp_seconds(msg.header.stamp)
            with self._lock:
                if stamp>=self._global_epoch and (self._particles_state is None or stamp>self._particles_state[0]):
                    self._particles_state=(stamp,particles)
        except (ValueError,TypeError):return

    def _odom(self, msg, info):
        try:
            p = msg.pose.pose
            pose = Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
            speed, turn = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            if not all(math.isfinite(v) for v in (speed, turn)):
                raise ValueError("invalid_odometry_velocity")
            with self._lock:
                if not msg.header.frame_id or not msg.child_frame_id:
                    raise ValueError("missing_odometry_frames")
                if self._buffer.samples and (msg.header.frame_id != self._odom_frame or msg.child_frame_id != self._base_frame):
                    raise ValueError("odometry_frame_changed")
                self._odom_frame, self._base_frame = msg.header.frame_id, msg.child_frame_id
                if not self._buffer.add(stamp_seconds(msg.header.stamp), pose):
                    return
                now = time.monotonic()
                self._odom_seen = now
                self._odom_source = endpoint_id(info)
                if not self._odom_source and self._node is not None:
                    publishers = self._node.get_publishers_info_by_topic(self._node.resolve_topic_name(self.odom_topic))
                    self._odom_source = endpoint_id(publishers[0]) if len(publishers) == 1 else ""
                if abs(speed) < .008 and abs(turn) < .025:
                    self._stationary_since = self._stationary_since or now
                else:
                    self._stationary_since = 0.
        except (ValueError, TypeError) as exc:
            with self._lock:
                self._error = str(exc)

    def _amcl(self, msg):
        try:
            if msg.header.frame_id.lstrip("/") != "map":
                raise ValueError("AMCL localization must use map frame")
            p, covariance = msg.pose.pose, msg.pose.covariance
            pose = Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
            variance = tuple(covariance[i] for i in (0, 7, 35))
            if not all(math.isfinite(v) and v >= 0 for v in variance):
                raise ValueError("invalid_localization_covariance")
            with self._lock:
                stamp = stamp_seconds(msg.header.stamp)
                if stamp<self._global_epoch:return
                if self._localization is None or stamp > self._localization[0]:
                    self._localization = (stamp, pose, variance)
                    self._pose_history.append((stamp,pose))
        except (ValueError, TypeError) as exc:
            with self._lock:
                self._error = str(exc)

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            ros_now = self._node.get_clock().now().nanoseconds*1e-9 if self._node is not None else time.time()
            odom_age = ros_now-self._buffer.samples[-1][0] if self._buffer.samples else math.inf
            local_age = ros_now-self._localization[0] if self._localization else math.inf
            stationary = bool(self._stationary_since and now-self._stationary_since >= .7)
            if stationary and self._buffer.samples:
                end_stamp, end_pose = self._buffer.samples[-1]
                recent = [p for t, p in self._buffer.samples if end_stamp-t <= .7]
                stationary = all(math.hypot(p.x-end_pose.x, p.y-end_pose.y) < .01 and abs(wrap(p.yaw-end_pose.yaw)) < .025 for p in recent)
            pose = self._localization[1] if self._localization else None
            variance = self._localization[2] if self._localization else None
            reason = self._error or "ready"
            if not self._error:
                if not self._buffer.samples or now-self._odom_seen > .4 or not -.06 <= odom_age <= .4:
                    reason = "waiting_fresh_odometry"
                elif self.auto_localize and not stationary:
                    reason='keep_robot_stationary'
                elif self.auto_localize and not -.06<=ros_now-self._scan_stamp<=.6:
                    reason='waiting_localization_scan'
                elif self.auto_localize and self._map_error:
                    reason=self._map_error
                elif self.auto_localize and self._global_request.is_set():
                    reason=self._global_status
                elif self.auto_localize and self._matching_status:
                    reason=self._matching_status
                elif self._localization is None or not -.06 <= local_age <= 5:
                    reason = "waiting_fresh_AMCL_pose"
                elif variance[0] > .0025 or variance[1] > .0025 or variance[2] > .0077:
                    reason = "localization_uncertain"
                elif not stationary:
                    reason = "keep_robot_stationary"
                else:
                    try:
                        at_local = self._buffer.at(self._localization[0])
                        pose = transform_pose(self._localization[1], at_local, self._buffer.samples[-1][1])
                    except ValueError as exc:
                        reason = str(exc)
            support=None
            from .localization import particle_support,stable_localization,STABILITY_WINDOW
            stable_recent=[(t,p) for t,p in self._pose_history
                if self._localization is not None and self._localization[0]-STABILITY_WINDOW<=t<=self._localization[0]]
            if self.auto_localize and reason=='ready':
                if self._particles_state is None or not -.06<=ros_now-self._particles_state[0]<=2.:
                    reason='waiting_localization_particles'
                else:
                    support=particle_support(pose,self._particles_state[1])
                    if support<.95:reason='localization_ambiguous'
                    elif not stable_localization(self._pose_history,self._localization[0]):reason='waiting_localization_stable'
                    elif self._matched_pose is not None and (math.hypot(pose.x-self._matched_pose.x,pose.y-self._matched_pose.y)>.15
                            or abs(wrap(pose.yaw-self._matched_pose.yaw))>.2):reason='scan_map_mismatch'
            blocking_reason=reason
            if (self.auto_localize and reason in ('waiting_fresh_AMCL_pose','localization_uncertain',
                    'localization_ambiguous','waiting_localization_stable','waiting_localization_particles',
                    'scan_map_mismatch','insufficient_localization_scan','no_localization_match','robot_moved_during_search')
                    and self._search_started and now-self._search_started>45):
                reason='automatic_localization_timeout'
            if reason == "ready" and self._node is not None:
                publishers = self._node.get_publishers_info_by_topic(self._node.resolve_topic_name(self.odom_topic))
                if len(publishers) != 1:
                    reason = "waiting_single_odometry_publisher"
                elif endpoint_id(publishers[0]) != self._odom_source:
                    reason = "odometry_publisher_changed"
            return dict(ready=reason == "ready", reason=reason,
                        blocking_reason=blocking_reason if reason=='automatic_localization_timeout' else reason,
                        scan_quality=self._scan_quality,pose=pose,covariance=variance,
                        odom=self._buffer.samples[-1][1] if self._buffer.samples else None,
                        odom_frame=self._odom_frame, base_frame=self._base_frame,
                        odom_age=odom_age, localization_age=local_age,
                        scan_age=ros_now-self._scan_stamp,map_ready=self._map_ready,
                        match_position_error=math.hypot(pose.x-self._matched_pose.x,pose.y-self._matched_pose.y)
                            if pose is not None and self._matched_pose is not None else None,
                        match_yaw_error=abs(wrap(pose.yaw-self._matched_pose.yaw))
                            if pose is not None and self._matched_pose is not None else None,
                        stationary=stationary, particle_support=support, scan_agreement=self._scan_agreement,
                        match_attempts=self._match_attempts,stability_samples=len(stable_recent),
                        stability_span=stable_recent[-1][0]-stable_recent[0][0] if stable_recent else 0.,
                        stability_position_drift=max((math.hypot(p.x-stable_recent[-1][1].x,p.y-stable_recent[-1][1].y)
                            for _,p in stable_recent),default=0.),
                        stability_yaw_drift=max((abs(wrap(p.yaw-stable_recent[-1][1].yaw))
                            for _,p in stable_recent),default=0.),error=self._error)

    def capture(self, arena, robot_id="tb3"):
        with self._lock:
            state = self.snapshot()
            if not state["ready"]:
                raise ValueError("Localization not ready: "+state["reason"])
            stamp, odom = self._buffer.samples[-1]
            return arena.pose_from_map(state["pose"]), InitialReference(
                mode="localized", robot_id=robot_id, odom_frame=self._odom_frame,
                base_frame=self._base_frame, odom_pose=odom, stamp=stamp, captured_at=time.time(),
                odom_source=self._odom_source)

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=2)


class RuntimeState:
    """ROS-independent handoff and sensor validity checks."""
    def __init__(self, knowledge, robot_id="tb3", drive=False):
        self.knowledge, self.reference, self.hardware = knowledge, knowledge.reference, knowledge.hardware
        self.buffer = PoseBuffer()
        self.odom_seen = self.scan_seen = 0.
        self.speed = 0.
        self.turn = 0.
        self.stopped = ""
        self.pose = knowledge.start
        self.anchored = False
        self._stationary_since = 0.
        if robot_id != self.reference.robot_id:
            raise ValueError("wrong_robot_identity")
        if drive and self.reference.mode == "simulation":
            raise ValueError("simulation knowledge cannot drive hardware; capture real start/odometry in Program 1")
        if drive and not self.reference.odom_source:
            raise ValueError("missing_odometry_source; recapture real start in Program 1")
        if drive and self.reference.odom_pose is None:
            raise ValueError("missing_real_odometry_reference")
        if drive and (time.time()-self.reference.captured_at > 1800 or self.reference.captured_at-time.time() > .1):
            raise ValueError("stale_initial_reference; repeat stationary capture in Program 1")

    def stop(self, reason):
        self.stopped = self.stopped or reason

    def odometry(self, stamp, pose, frame, child, speed, turn, ros_now, monotonic=None):
        now = time.monotonic() if monotonic is None else monotonic
        if frame.lstrip("/") != self.reference.odom_frame.lstrip("/") or child.lstrip("/") != self.reference.base_frame.lstrip("/"):
            raise ValueError("odometry_frame_mismatch")
        if not all(math.isfinite(v) for v in (speed, turn, ros_now)):
            raise ValueError("invalid_odometry_velocity")
        if not math.isfinite(stamp) or stamp<=0:
            raise ValueError('invalid_odometry_stamp')
        if ros_now-stamp>self.hardware.odom_timeout:
            # Old DDS samples must not anchor a handoff or refresh the
            # watchdog. A later fresh sample can complete startup normally.
            return False
        if ros_now-stamp < -.06:
            raise ValueError("stale_or_future_odometry")
        if not self.anchored:
            reference = self.reference.odom_pose
            if reference is not None:
                if stamp < self.reference.stamp-.06:
                    raise ValueError("odometry_reference_time_reset")
                if math.hypot(pose.x-reference.x, pose.y-reference.y) > .025 or abs(wrap(pose.yaw-reference.yaw)) > .08:
                    raise ValueError("robot_moved_since_capture_or_odometry_reset")
            if abs(speed) > .008 or abs(turn) > .025:
                raise ValueError("robot_must_start_stationary")
            self.anchored = True
        if not self.buffer.add(stamp, pose):
            return False
        self.odom_seen = now
        self.speed, self.turn = speed, turn
        if abs(speed) < .008 and abs(turn) < .025:
            self._stationary_since = self._stationary_since or now
        else:
            self._stationary_since = 0.
        ref = self.reference.odom_pose or self.buffer.samples[0][1]
        self.pose = transform_pose(self.knowledge.start, ref, pose)
        return True

    def ready_to_arm(self, monotonic=None):
        now = time.monotonic() if monotonic is None else monotonic
        if not self._stationary_since or now-self._stationary_since < .7 or not self.buffer.samples:
            return False
        stamp, current = self.buffer.samples[-1]
        reference = self.reference.odom_pose
        if reference is not None and (math.hypot(current.x-reference.x, current.y-reference.y) > .025
                                      or abs(wrap(current.yaw-reference.yaw)) > .08):
            self.stop("robot_moved_since_capture")
            return False
        return all(math.hypot(p.x-current.x, p.y-current.y) < .01 and abs(wrap(p.yaw-current.yaw)) < .025
                   for t, p in self.buffer.samples if stamp-t <= .7)

    def health(self, ros_now, monotonic=None):
        now = time.monotonic() if monotonic is None else monotonic
        if self.stopped:
            return self.stopped
        if not self.anchored:
            return "waiting_odometry"
        if now-self.odom_seen > self.hardware.odom_timeout or not -.06 <= ros_now-self.buffer.samples[-1][0] <= self.hardware.odom_timeout:
            return "odometry_timeout"
        if now-self.scan_seen > self.hardware.scan_timeout:
            return "scan_timeout"
        return ""


def _make_navigation_node(knowledge, *, drive, scan_topic, odom_topic, command_topic,
                          robot_id, log_path=None, context=None,ui_stream=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped, PoseStamped, TransformStamped, Point
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_msgs.msg import TFMessage
    import tf2_ros
    from .navigation import Navigator, Observation

    state = RuntimeState(knowledge, robot_id, drive)
    class NavigationNode(Node):
        def __init__(self):
            super().__init__("knowledge_navigation", context=context)
            self.state = state
            self.odom_topic = self.resolve_topic_name(odom_topic)
            self.navigator = Navigator(knowledge.arena, knowledge.goal, knowledge.policy, knowledge.hardware)
            self.command_pub = self.create_publisher(TwistStamped, command_topic, 10) if drive else None
            self.status_pub = self.create_publisher(String, "/knowledge/status", 10)
            self.pose_pub = self.create_publisher(PoseStamped, "/knowledge/pose", 10)
            self.path_pub = self.create_publisher(RosPath, "/knowledge/local_path", 10)
            self.trail_pub = self.create_publisher(RosPath, "/knowledge/trail", 10)
            self.markers_pub = self.create_publisher(MarkerArray, "/knowledge/markers", 10)
            self.create_subscription(Odometry, odom_topic, self.odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, scan_topic, self.scan, qos_profile_sensor_data)
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=12))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.create_subscription(TFMessage, "/tf", self.tf_check, qos_profile_sensor_data)
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, "/tf_static", self.tf_check, static_qos)
            self.create_subscription(String, command_topic+"/guard_status", self.guard_status, 10)
            self.create_service(Trigger, "~/stop", self.stop_request)
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(knowledge.hardware.control_period, self.tick, clock=self.steady)
            self.latest_scan = None
            self.scan_history = deque(maxlen=32)
            self.selected_scan_stamp = None
            self.last_reported_reason = None
            self.reason = "waiting_sensors"
            self.sensor_fault_info=None
            self.discarded_old_scans=0
            self.last_discarded_scan_age=None
            self.display_observation=None
            self.last_ui_update=0.
            self.traveled_distance=0.
            self.started = time.monotonic()
            self.last_tick = self.started
            self.driving_started = False
            self.guard_reason = "waiting_guard" if drive else "dry_run"
            self._guard_ready = False
            self._guard_seen = 0.
            self.trail = deque(maxlen=2000)
            self._last_trail_time = 0.
            self._last_graph_check = 0.
            self._full_map_active = False
            self._full_map_present=None
            self._odom_publisher_count=None
            self._odom_publisher_matches=None
            self._condition_key=None
            self.movement_conditions=[]
            self._conditions_checked=0.
            self.observation_error=None
            self.observation_checked_at=None
            self.log = None
            if log_path:
                path = Path(log_path).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                self.log = path.open("a", encoding="utf-8")
            self.get_logger().info("DRIVE ENABLED: starts after sensor/guard checks" if drive else "DRY RUN: no velocity publisher exists")

        def ros_now(self):
            return self.get_clock().now().nanoseconds*1e-9

        def guard_status(self, msg):
            self.guard_reason = msg.data
            self._guard_seen = time.monotonic()
            if msg.data in ("forwarding", "command_timeout"):
                self._guard_ready = True
            if msg.data in ("competing_motor_publisher", "competing_requested_publisher", "motor_topic_type_mismatch", "operator_stop", "invalid_command", "command_frame_mismatch"):
                state.stop("guard_"+msg.data)

        def stop_request(self, request, response):
            state.stop("operator_stop")
            self.send(0., 0.)
            response.success, response.message = True, "Stopped; repeat setup before next mission"
            return response

        def tf_check(self, msg):
            # Any external parent correcting odom is map/externally corrected
            # assistance. Our fixed field frame is a child of odom instead.
            if not drive:
                return
            for transform in msg.transforms:
                if transform.child_frame_id.lstrip("/") == state.reference.odom_frame.lstrip("/"):
                    self._full_map_active = True
                    state.stop("full_map_localization_active")

        def odom(self, msg, info):
            try:
                if drive:
                    source = endpoint_id(info)
                    publishers = self.get_publishers_info_by_topic(self.odom_topic)
                    if not publishers and not state.anchored:
                        return  # wait for graph discovery before anchoring
                    if len(publishers) != 1 or endpoint_id(publishers[0]) != state.reference.odom_source:
                        raise ValueError("odometry_publisher_changed")
                    if source and source != state.reference.odom_source:
                        raise ValueError("odometry_publisher_changed")
                p, t = msg.pose.pose, msg.twist.twist
                pose = Pose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
                previous=state.pose
                if state.odometry(stamp_seconds(msg.header.stamp), pose, msg.header.frame_id,
                               msg.child_frame_id, t.linear.x, t.angular.z, self.ros_now()):
                    self.traveled_distance+=math.hypot(state.pose.x-previous.x,state.pose.y-previous.y)
            except (ValueError, TypeError) as exc:
                if not state.stopped:
                    self.sensor_fault_info=dict(stream='odom',age=self.ros_now()-stamp_seconds(msg.header.stamp),reason=str(exc))
                    if str(exc)=='robot_moved_since_capture_or_odometry_reset' and state.reference.odom_pose is not None:
                        reference=state.reference.odom_pose
                        self.sensor_fault_info.update(reference_distance=math.hypot(pose.x-reference.x,pose.y-reference.y),
                            reference_yaw_degrees=math.degrees(wrap(pose.yaw-reference.yaw)))
                state.stop(str(exc))

        def scan(self, msg):
            try:
                stamp = stamp_seconds(msg.header.stamp)
                age=self.ros_now()-stamp
                if not math.isfinite(stamp) or stamp<=0:
                    raise ValueError('malformed_scan')
                if age>knowledge.hardware.scan_timeout:
                    # Discovery/startup can deliver an old queued scan before
                    # fresh ones. Discard it without touching the accepted
                    # scan or its watchdog; sustained loss still stops motion.
                    self.discarded_old_scans+=1
                    self.last_discarded_scan_age=age
                    return
                if age < -.06:
                    raise ValueError("stale_or_future_scan")
                if (not msg.header.frame_id or not math.isfinite(msg.angle_min)
                    or not math.isfinite(msg.angle_increment) or msg.angle_increment == 0
                    or not math.isfinite(msg.time_increment) or msg.time_increment < 0
                    or not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max)
                    or msg.range_min < 0 or msg.range_max <= msg.range_min
                    or len(msg.ranges) < 16
                    or len(msg.ranges)*abs(msg.angle_increment)>2*math.pi+.15
                    or msg.time_increment*(len(msg.ranges)-1) > knowledge.hardware.scan_timeout):
                    raise ValueError("malformed_scan")
                if len(msg.ranges)*abs(msg.angle_increment)<2*math.pi-.15:
                    # A clipped but well-formed sweep is a temporary dropout.
                    # Keep the previous complete sweep only while still fresh;
                    # this frame never refreshes the scan watchdog.
                    self.last_scan_rejection='incomplete_scan'
                    return
                if self.latest_scan is not None and stamp <= stamp_seconds(self.latest_scan.header.stamp):
                    return
                self.latest_scan = msg
                self.scan_history.append(msg)
                state.scan_seen = time.monotonic()
            except (ValueError, TypeError) as exc:
                if not state.stopped:
                    self.sensor_fault_info=dict(stream='scan',age=self.ros_now()-stamp_seconds(msg.header.stamp),reason=str(exc))
                state.stop(str(exc))

        def observation(self):
            self.selected_scan_stamp=None
            self.filled_scan_gaps=0
            self.unknown_scan_sectors=None
            scan = complete_scan(self.scan_history,state.buffer,self.ros_now(),knowledge.hardware.scan_timeout)
            stamp = stamp_seconds(scan.header.stamp)
            self.selected_scan_stamp=stamp
            base, laser = state.reference.base_frame, scan.header.frame_id
            if laser.lstrip("/") in (state.reference.odom_frame.lstrip("/"), "map", "knowledge_field"):
                raise ValueError("invalid_lidar_frame")
            if base.lstrip("/") == laser.lstrip("/"):
                mount = (0., 0., 0.)
            else:
                try:
                    transform = self.tf_buffer.lookup_transform(base, laser, Time.from_msg(scan.header.stamp)).transform
                except Exception as exc:
                    raise ValueError("waiting_lidar_TF") from exc
                translation = transform.translation
                if (not all(math.isfinite(x) for x in (translation.x, translation.y, translation.z))
                    or abs(translation.z) > .5 or math.hypot(translation.x, translation.y) > .35):
                    raise ValueError("invalid_lidar_mount")
                mount = (transform.translation.x, transform.translation.y, yaw_from_quaternion(transform.rotation))
                if not all(math.isfinite(x) for x in mount):
                    raise ValueError("invalid_lidar_mount")
            observation=aligned_scan_observation(scan,self.scan_history,state.buffer,state.pose,
                state.speed,mount,self.ros_now(),knowledge.hardware,angular_speed=state.turn)
            self.selected_scan_stamp=observation.stamp
            self.filled_scan_gaps=sum(not math.isfinite(value) or not scan.range_min<=value<=scan.range_max
                for value in scan.ranges)-len(observation.invalid_angles)-observation.interpolated_beams
            self.interpolated_scan_gaps=observation.interpolated_beams
            self.unknown_scan_sectors=len(observation.invalid_angles)
            self.display_observation=observation
            return observation

        def full_map_check(self):
            names = self.get_node_names_and_namespaces()
            if any(any(token in name.lower() for token in ("amcl", "cartographer", "slam_toolbox")) for name, namespace in names):
                return True
            return bool(self.get_publishers_info_by_topic("/amcl_pose"))

        def tick(self):
            monotonic, now = time.monotonic(), self.ros_now()
            elapsed = monotonic-self.last_tick
            self.last_tick = monotonic
            v = w = 0.
            issue = state.health(now)
            if drive and monotonic-self._last_graph_check > .25:
                self._last_graph_check = monotonic
                self._full_map_present=self.full_map_check()
                if self._full_map_present:
                    state.stop("full_map_localization_active")
                    issue = state.stopped
                publishers = self.get_publishers_info_by_topic(self.odom_topic)
                self._odom_publisher_count=len(publishers)
                self._odom_publisher_matches=len(publishers)==1 and endpoint_id(publishers[0])==state.reference.odom_source
                if len(publishers) > 1 or publishers and endpoint_id(publishers[0]) != state.reference.odom_source:
                    state.stop("odometry_publisher_changed")
                    issue = state.stopped
                if self.driving_started and not publishers:
                    state.stop("odometry_publisher_lost")
                    issue = state.stopped
            if monotonic-self.started > 180:
                state.stop("mission_timeout")
                issue = state.stopped
            if drive and self.driving_started and monotonic-self._guard_seen > max(.15, knowledge.hardware.command_timeout):
                state.stop("guard_status_timeout")
                issue = state.stopped
            if drive and elapsed > knowledge.hardware.command_timeout:
                state.stop("control_loop_overrun")
                issue = state.stopped
            if issue and self.driving_started and issue in ("scan_timeout", "odometry_timeout"):
                state.stop(issue)
            if not issue:
                try:
                    self.observation_checked_at=now
                    obs = self.observation()
                    self.observation_error=''
                    enabled = True
                    if drive and not self.driving_started:
                        if not state.ready_to_arm(monotonic):
                            issue, enabled = "keep_robot_stationary", False
                        elif not self._guard_ready or monotonic-self._guard_seen > knowledge.hardware.command_timeout or monotonic-self.started < 1.5:
                            issue, enabled = "waiting_guard_and_graph", False
                        else:
                            self.driving_started = True
                    if enabled:
                        v, w = self.navigator.command(obs, now)
                        issue = self.navigator.reason
                    if self.navigator.reason in ("arrived_estimate", "recovery_exhausted", "outside_safe_arena"):
                        state.stop(self.navigator.reason)
                        v = w = 0.
                except ValueError as exc:
                    self.observation_error=str(exc)
                    issue, v, w = str(exc), 0., 0.
            if state.stopped:
                issue, v, w = state.stopped, 0., 0.
            if drive and self.guard_reason not in ("forwarding", "command_timeout"):
                v = w = 0.
            self.reason = issue or "running"
            if drive and v == 0. and w == 0.:
                self.navigator.pause()
            self.send(v, w)
            self.visualize(now)
            quality=scan_coverage(self.latest_scan)
            arm_stable=arm_reference=None
            if drive and not self.driving_started and state.buffer.samples:
                stamp,current=state.buffer.samples[-1]
                arm_stable=all(math.hypot(p.x-current.x,p.y-current.y)<.01 and abs(wrap(p.yaw-current.yaw))<.025
                    for t,p in state.buffer.samples if stamp-t<=.7)
                ref=state.reference.odom_pose
                if ref is not None:
                    arm_reference=math.hypot(current.x-ref.x,current.y-ref.y)<=.025 and abs(wrap(current.yaw-ref.yaw))<=.08
            status = dict(mode=self.navigator.mode, reason=self.reason, drive=drive,
                          stop_latched=bool(state.stopped),
                          guard=self.guard_reason, pose=asdict(state.pose), requested=[v, w],
                          scan_age=now-self.selected_scan_stamp if self.selected_scan_stamp is not None else now-stamp_seconds(self.latest_scan.header.stamp) if self.latest_scan is not None else None,
                          odom_age=now-state.buffer.samples[-1][0] if state.buffer.samples else None,
                          filled_scan_gaps=max(0,getattr(self,'filled_scan_gaps',0)),
                          interpolated_scan_gaps=getattr(self,'interpolated_scan_gaps',0),
                          unknown_scan_sectors=getattr(self,'unknown_scan_sectors',None),
                          sensor_fault=self.sensor_fault_info,
                          discarded_old_scans=self.discarded_old_scans,
                          last_discarded_scan_age=self.last_discarded_scan_age,
                          estimated_goal_error=math.hypot(state.pose.x-knowledge.goal[0], state.pose.y-knowledge.goal[1]),
                          recoveries=self.navigator.recoveries, fingerprint=knowledge.fingerprint,
                          scan_message_age=now-stamp_seconds(self.latest_scan.header.stamp) if self.latest_scan is not None else None,
                          scan_required_coverage_percent=MIN_SCAN_COVERAGE_PERCENT,
                          scan_coverage_passed=None if quality['scan_coverage_percent'] is None else quality['scan_coverage_percent']>=MIN_SCAN_COVERAGE_PERCENT,
                          anchored=state.anchored,
                          odom_receive_age=monotonic-state.odom_seen if state.odom_seen else None,
                          scan_span=len(self.latest_scan.ranges)*abs(self.latest_scan.angle_increment) if self.latest_scan is not None else None,
                          runtime_elapsed=monotonic-self.started,control_loop_elapsed=elapsed,
                          odom_publisher_count=self._odom_publisher_count,odom_publisher_matches=self._odom_publisher_matches,
                          full_map_present=self._full_map_present,drive_armed=self.driving_started,
                          arm_pose_stable=arm_stable,arm_reference_matches=arm_reference,
                          stationary_duration=monotonic-state._stationary_since if state._stationary_since else 0.,
                          guard_status_age=monotonic-self._guard_seen if self._guard_seen else None,
                          observation_error=self.observation_error,
                          observation_error_age=now-self.observation_checked_at if self.observation_checked_at is not None else None,
                          **quality)
            condition_key=(self.reason,self.guard_reason,bool(state.stopped),int(monotonic))
            if condition_key!=self._condition_key:
                from .diagnostics import movement_conditions,format_movement_conditions
                self.movement_conditions=movement_conditions(status,self.navigator,self.display_observation,now)
                self._conditions_checked=monotonic
                self._condition_key=condition_key
                pending=format_movement_conditions(self.movement_conditions,failed_only=True)
                print('TB3_CHECKS: เงื่อนไขที่ไม่ผ่าน / ยังตรวจไม่ได้ (แยกตามการเคลื่อนที่)\n'+(pending or 'ผ่านเงื่อนไขที่ตรวจได้ทั้งหมด'),flush=True)
            status['movement_conditions']=self.movement_conditions
            status['conditions_age']=monotonic-self._conditions_checked
            message = String()
            message.data = json.dumps(status, allow_nan=False)
            self.status_pub.publish(message)
            if ui_stream and monotonic-self.last_ui_update>=.2:
                self.last_ui_update=monotonic
                obs=self.display_observation
                points=obs.points if obs is not None else ()
                stride=max(1,math.ceil(len(points)/500))
                data=dict(status,elapsed=monotonic-self.started,distance=self.traveled_distance,
                    explanation=runtime_status_text(self.reason,drive),
                    path=self.navigator.local_path,points=points[::stride],
                    sensor_pose=asdict(obs.pose) if obs is not None else asdict(state.pose),
                    unknown=[asdict(sector) for sector in (obs.unknown_sectors or ())] if obs is not None else [])
                print('TB3_DATA: '+json.dumps(data,allow_nan=False),flush=True)
            report_key=(self.reason,self.guard_reason,None if ui_stream else int(monotonic))
            if report_key!=self.last_reported_reason:
                description=runtime_status_text(self.reason,drive)
                if self.sensor_fault_info is not None and self.reason in ('stale_or_future_scan','stale_or_future_odometry'):
                    age=self.sensor_fault_info['age']
                    description+=f' (timestamp เร็วกว่าคอม {-age:.3f} s)' if age<0 else f' (ข้อมูลล่าช้า {age:.3f} s)'
                if self.sensor_fault_info is not None and 'reference_yaw_degrees' in self.sensor_fault_info:
                    description+=f" (ต่าง {self.sensor_fault_info['reference_distance']:.3f} m, {self.sensor_fault_info['reference_yaw_degrees']:.1f}°)"
                if self.guard_reason=='motor_topic_type_mismatch':
                    description+=' · ชนิด /cmd_vel ไม่ตรงกับหุ่น เลือก auto แล้วเริ่มใหม่'
                if self.reason in ('no_feasible_gap','unknown_rotation_clearance'):
                    unknown=getattr(self,'unknown_scan_sectors',None)
                    if unknown is not None:description+=f' (ยังไม่มีข้อมูล {unknown} ช่องมุม)'
                coverage=status['scan_coverage_percent']
                if coverage is not None:
                    description+=f" · สแกนล่าสุดอ่านได้ {status['scan_valid_beams']}/{status['scan_total_beams']} ({coverage:.1f}%)"
                    description+=f" · ขาด {status['scan_missing_percent']:.1f}% · อายุ {status['scan_message_age']:.3f} s"
                    description+=f" · เกณฑ์ {MIN_SCAN_COVERAGE_PERCENT:.0f}%: "+('ผ่าน' if status['scan_coverage_passed'] else 'ไม่ผ่าน')
                print('TB3_STATUS: '+description,flush=True)
                self.last_reported_reason=report_key
            if self.log is not None:
                self.log.write(json.dumps(dict(time=now, **status), allow_nan=False)+"\n")
                self.log.flush()

        def send(self, v, w):
            if self.command_pub is not None:
                command = TwistStamped()
                command.header.stamp = self.get_clock().now().to_msg()
                command.header.frame_id = state.reference.base_frame
                command.twist.linear.x, command.twist.angular.z = float(v), float(w)
                self.command_pub.publish(command)

        def stamped_pose(self, pose):
            message = PoseStamped()
            message.header.frame_id = "knowledge_field"
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y = float(pose.x), float(pose.y)
            message.pose.orientation.z = math.sin(pose.yaw/2)
            message.pose.orientation.w = math.cos(pose.yaw/2)
            return message

        def visualize(self, now):
            message = self.stamped_pose(state.pose)
            self.pose_pub.publish(message)
            if now-self._last_trail_time > .15:
                self.trail.append(message)
                self._last_trail_time = now
            local = RosPath()
            local.header = message.header
            local.poses = [self.stamped_pose(Pose(float(point[0]), float(point[1]))) for point in self.navigator.local_path]
            self.path_pub.publish(local)
            trail = RosPath()
            trail.header, trail.poses = message.header, list(self.trail)
            self.trail_pub.publish(trail)
            markers = MarkerArray()
            for index, (label, position, color) in enumerate((("Start", knowledge.start, (0., 1., 0.)),
                    ("Goal", Pose(*knowledge.goal), (1., .3, 0.)), ("Robot", state.pose, (.2, .5, 1.)))):
                marker = Marker()
                marker.header, marker.ns, marker.id = message.header, "mission", index
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose = self.stamped_pose(position).pose
                marker.scale.x = marker.scale.y = .10 if label != "Robot" else knowledge.hardware.radius*2
                marker.scale.z = .05
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.
                markers.markers.append(marker)
                text = Marker()
                text.header, text.ns, text.id = message.header, "labels", index
                text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                text.pose = self.stamped_pose(position).pose
                text.pose.position.z = .18
                text.scale.z = .12
                text.color.r = text.color.g = text.color.b = text.color.a = 1.
                text.text = label
                markers.markers.append(text)
            border = Marker()
            border.header, border.ns, border.id = message.header, "boundary", 0
            border.type, border.action = Marker.LINE_STRIP, Marker.ADD
            border.pose.orientation.w = 1.
            border.scale.x = .025
            border.color.r = border.color.g = border.color.b = .8
            border.color.a = 1.
            for x, y in ((0, 0), (knowledge.arena.width, 0), (knowledge.arena.width, knowledge.arena.height), (0, knowledge.arena.height), (0, 0)):
                p = Point()
                p.x, p.y = float(x), float(y)
                border.points.append(p)
            markers.markers.append(border)
            status_marker = Marker()
            status_marker.header, status_marker.ns, status_marker.id = message.header, "status", 0
            status_marker.type, status_marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            status_marker.pose.position.x = knowledge.arena.width/2
            status_marker.pose.position.y = knowledge.arena.height+.15
            status_marker.pose.position.z = .12
            status_marker.pose.orientation.w = 1.
            status_marker.scale.z = .12
            status_marker.color.r = status_marker.color.g = status_marker.color.b = status_marker.color.a = 1.
            status_marker.text = ("DRIVE" if drive else "DRY RUN")+" | "+self.navigator.mode+"\n"+self.reason
            markers.markers.append(status_marker)
            self.markers_pub.publish(markers)
            if state.buffer.samples:
                ref = state.reference.odom_pose or state.buffer.samples[0][1]
                # odom -> field preserves the driver tree even during setup
                # dry runs; field -> odom would compete with AMCL map -> odom.
                anchor = compose(ref, inverse(knowledge.start))
                tf = TransformStamped()
                tf.header.stamp = message.header.stamp
                tf.header.frame_id = state.reference.odom_frame
                tf.child_frame_id = "knowledge_field"
                tf.transform.translation.x, tf.transform.translation.y = anchor.x, anchor.y
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(anchor.yaw/2), math.cos(anchor.yaw/2)
                self.tf_broadcaster.sendTransform(tf)

        def cleanup(self):
            # Executor flushing may run another timer. Latch before zeroing.
            state.stop("shutdown")
            self.navigator.pause()
            for _ in range(3):
                self.send(0., 0.)
            if self.log is not None:
                self.log.close()
                self.log = None
    return NavigationNode()


def run_navigation(knowledge, *, drive=False, output_type="auto", scan_topic="/scan",
                   odom_topic="/odom", command_topic="/knowledge/cmd_vel", output_topic="/cmd_vel",
                   robot_id="tb3", launch_rviz=False, log_path=None,ui_stream=False):
    """Blocking runtime. Dry run creates no requested/motor velocity publisher."""
    # Reject unsafe reference before creating a managed motor-publishing process.
    if log_path is not None and Path(log_path).suffix.lower() != ".jsonl":
        raise ValueError("ROS log filename must end in .jsonl")
    RuntimeState(knowledge, robot_id, drive)
    if output_type not in ("auto", "stamped", "twist"):
        raise ValueError("output_type must be auto, stamped or twist")
    if command_topic.rstrip("/") == output_topic.rstrip("/"):
        raise ValueError("requested and motor topics must differ")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    guard = rviz = node = executor = None
    previous = {}
    stop_requested = threading.Event()
    def interrupted(signum, frame):
        stop_requested.set()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        previous[sig] = signal.signal(sig, interrupted)
    try:
        node = _make_navigation_node(knowledge, drive=drive, scan_topic=scan_topic,
                odom_topic=odom_topic, command_topic=command_topic, robot_id=robot_id,
                log_path=log_path, context=context,ui_stream=ui_stream)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        if drive:
            from .guard import motor_message_type
            output=node.resolve_topic_name(output_topic)
            deadline=time.monotonic()+4.
            detected=None
            print('TB3_STATUS: กำลังตรวจชนิดคำสั่งที่ driver หุ่นรับ ไม่ส่งคำสั่งมอเตอร์ระหว่างตรวจ',flush=True)
            while not stop_requested.is_set() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.05)
                detected=motor_message_type(node.get_subscriptions_info_by_topic(output),output_type)
                if detected is not None:break
            if stop_requested.is_set():return 0
            if detected is None:raise ValueError('waiting_motor_subscriber: no driver subscribed to '+output)
            output_type=detected
            print(f'TB3_STATUS: driver รับ {output_type} ตรวจพบชนิดคำสั่งอัตโนมัติแล้ว',flush=True)
            h = knowledge.hardware
            guard = subprocess.Popen([sys.executable, str(Path(__file__).with_name("guard.py")),
                "--input-topic", command_topic, "--output-topic", output_topic,
                "--output-type", output_type, "--max-speed", str(h.max_speed),
                "--max-angular", str(h.max_angular_speed), "--timeout", str(h.command_timeout),
                "--base-frame", knowledge.reference.base_frame])
        if launch_rviz:
            config = Path(__file__).resolve().parents[1]/"navigation.rviz"
            rviz = subprocess.Popen(["rviz2", "-d", str(config)])
        while rclpy.ok(context=context) and not stop_requested.is_set():
            executor.spin_once(timeout_sec=.05)
            if guard is not None and guard.poll() is not None:
                node.state.stop("guard_process_exited")
                node.send(0., 0.)
                break
    except KeyboardInterrupt:
        pass
    finally:
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            node.cleanup()
            deadline = time.monotonic() + .15
            while time.monotonic() < deadline:
                try:
                    executor.spin_once(timeout_sec=.025)
                except Exception:
                    break
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
            node.destroy_node()
        if guard is not None and guard.poll() is None:
            guard.terminate()
            try:
                guard.wait(timeout=2)
            except subprocess.TimeoutExpired:
                guard.kill()
                guard.wait()
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:
                rviz.wait(timeout=2)
            except subprocess.TimeoutExpired:
                rviz.kill()
                rviz.wait()
        if context.ok():
            context.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0
