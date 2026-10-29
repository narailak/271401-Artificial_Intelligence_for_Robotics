"""Read-only movement checks; alternative actions are reported separately."""
import math

from .clearance import UnknownSector,sector_distance
from .navigation import Navigator,MIN_SCAN_COVERAGE_PERCENT


def movement_conditions(status,navigator,observation,now):
    h=navigator.hardware
    checks=[]
    def check(code,label,passed,detail,scope='ระบบ'):
        checks.append(dict(code=code,label=label,passed=passed,detail=detail,scope=scope))
    def age(code,label,value,limit):
        check(code,label,None if value is None else -.06<=value<=limit,
            'ยังไม่มีข้อมูล' if value is None else f'{value:.3f} s; ต้อง -0.060 ถึง {limit:.3f} s')
    check('stop_latch','ไม่มีเหตุล็อกหยุด',not status.get('stop_latched',False),
        status.get('reason','') if status.get('stop_latched') else 'ไม่ล็อกหยุด')
    anchored=status.get('anchored',False)
    fault=status.get('sensor_fault') or {}
    detail='รับ odometry อ้างอิงแล้ว' if anchored else fault.get('reason','รอรับ odometry อ้างอิง')
    if 'reference_yaw_degrees' in fault:
        detail+=f"; ต่าง {fault['reference_distance']:.3f} m / {fault['reference_yaw_degrees']:.1f}°; ต้อง ≤0.025 m / {math.degrees(.08):.1f}°"
    check('reference','Start/reference ตรงกับ odometry',True if anchored else False if fault else None,detail)
    if fault:
        check('sensor_fault','ไม่มีความผิดปกติเซนเซอร์ที่บันทึกไว้',False,fault.get('reason','sensor_fault'))
    age('odom_stamp','อายุ odometry',status.get('odom_age'),h.odom_timeout)
    age('odom_delivery','ช่วงขาดการรับ odometry',status.get('odom_receive_age'),h.odom_timeout)
    age('raw_scan','อายุสแกนล่าสุด',status.get('scan_message_age'),h.scan_timeout)
    observation_age=status.get('observation_error_age')
    observation_error=status.get('observation_error')
    recent_check=observation_age is not None and -.06<=observation_age<=h.scan_timeout
    check('observation','TF/odometry ครอบคลุมสแกนและสร้าง observation ได้',None if not recent_check else observation_error=='',
        'ยังไม่มีผลตรวจล่าสุด' if not recent_check else observation_error or 'สร้าง observation ได้')
    coverage=status.get('scan_coverage_percent')
    check('raw_coverage','ความครบสแกนล่าสุด',None if coverage is None else coverage>=MIN_SCAN_COVERAGE_PERCENT,
        'ยังไม่มีข้อมูล' if coverage is None else f'{coverage:.1f}%; ต้อง ≥{MIN_SCAN_COVERAGE_PERCENT:.0f}%')
    span=status.get('scan_span')
    check('scan_span','ช่วงมุมสแกนครบรอบ',None if span is None else 2*math.pi-.15<=span<=2*math.pi+.15,
        'ยังไม่มีข้อมูล' if span is None else f'{math.degrees(span):.1f}°; ต้อง {math.degrees(2*math.pi-.15):.1f}–{math.degrees(2*math.pi+.15):.1f}°')
    elapsed=status.get('runtime_elapsed',0.)
    check('mission_time','เวลาภารกิจ',elapsed<=180.,f'{elapsed:.1f} s; ต้อง ≤180 s')
    if status.get('drive'):
        count=status.get('odom_publisher_count')
        matches=status.get('odom_publisher_matches')
        check('odom_source','ผู้ส่ง odometry เดียวและตรง reference',None if count is None else count==1 and matches,
            'รอ ROS graph' if count is None else f'{count} ผู้ส่ง; identity ตรง={matches}')
        present=status.get('full_map_present')
        check('localization_closed','ปิด AMCL/SLAM ก่อนเดิน',None if present is None else not present,
            'รอ ROS graph' if present is None else f'พบ AMCL/SLAM={present}')
        guard=status.get('guard','waiting_guard')
        check('guard','guard ส่งคำสั่งได้',guard=='forwarding',guard)
        age('guard_age','อายุสถานะ guard',status.get('guard_status_age'),max(.15,h.command_timeout))
        duration=status.get('control_loop_elapsed')
        check('control_loop','รอบคำนวณไม่ค้าง',None if duration is None else duration<=h.command_timeout,
            'ยังไม่มีข้อมูล' if duration is None else f'{duration:.3f} s; ต้อง ≤{h.command_timeout:.3f} s')
        if not status.get('drive_armed'):
            still=status.get('stationary_duration',0.)
            check('stationary','หุ่นนิ่งก่อนเริ่ม',still>=.7,f'{still:.2f} s; ต้อง ≥0.70 s')
            check('discovery_settled','รอ ROS graph ก่อนเริ่ม',elapsed>=1.5,f'{elapsed:.2f} s; ต้อง ≥1.50 s')
            check('arm_pose_stable','ตำแหน่ง odometry นิ่งก่อนเริ่ม',status.get('arm_pose_stable'),
                'ต้องแกว่ง <0.010 m และ <0.025 rad ในช่วง 0.7 s')
            check('arm_reference','ตำแหน่งก่อนเดินยังตรง Start/reference',status.get('arm_reference_matches'),
                'ต้องห่าง reference ≤0.025 m และ ≤0.080 rad')
    fresh=observation is not None and -.06<=now-observation.stamp<=h.scan_timeout
    age('navigation_scan','อายุหลักฐานสแกนที่ตรวจทาง',None if observation is None else now-observation.stamp,h.scan_timeout)
    if not fresh:
        for code,label,scope in (('arena','วงปลอดภัยอยู่ในกรอบสนาม','ระบบ'),
                ('navigation_coverage','ความครบสแกนที่ใช้ตรวจทาง','ระบบ'),
                ('footprint','ระยะรอบตัวหุ่น','ระบบ'),('rotation','พื้นที่หมุน','หมุน'),
                ('rotation_drift','จุดวัดจริงและขอบสนามพอสำหรับหยุดไหล','หมุน'),
                ('forward','ทางตรงก้าวตรวจ','ก้าวตรวจ'),('braking','ระยะเบรกก้าวตรวจ','ก้าวตรวจ'),
                ('route','ช่องทางเดินตามปกติ','เดินปกติ'),('trajectory','แนวเคลื่อนที่และระยะหยุดที่เลือก','เดินปกติ')):
            check(code,label,None,'ยังไม่มี observation สด; รอ reference/odom/TF/ประวัติสแกน',scope)
        return checks
    obs=observation
    probe=Navigator(navigator.arena,navigator.goal,navigator.policy,h)
    increment=obs.angle_increment or 2*math.pi/max(1,len(obs.ranges))
    probe._beam_width=max(abs(increment),obs.beam_spacing or 0.)
    points,invalid=obs.sensor_points(h.scan_range)
    sectors=list(obs.unknown_sectors) if obs.unknown_sectors is not None else [
        UnknownSector(obs.mount[:2],angle,probe._beam_width/2,h.lidar_min) for angle in invalid]
    used_coverage=obs.scan_coverage_percent
    if used_coverage is None:
        used_coverage=100*sum(math.isfinite(value) and 0<value<=h.scan_range+.05 for value in obs.ranges)/max(1,len(obs.ranges))
    check('navigation_coverage','ความครบสแกนที่ใช้ตรวจทาง',used_coverage>=MIN_SCAN_COVERAGE_PERCENT,
        f'{used_coverage:.1f}%; ต้อง ≥{MIN_SCAN_COVERAGE_PERCENT:.0f}%')
    if used_coverage>=MIN_SCAN_COVERAGE_PERCENT:
        sectors=[]
    check('arena','วงปลอดภัยอยู่ในกรอบสนาม',probe._inside(obs.pose.x,obs.pose.y),
        f'x={obs.pose.x:.3f}, y={obs.pose.y:.3f} m; รัศมีรวม {probe.radius:.3f} m')
    goal_distance=math.hypot(navigator.goal[0]-obs.pose.x,navigator.goal[1]-obs.pose.y)
    check('goal','ยังต้องเคลื่อนที่ไป Goal',goal_distance>.8*h.goal_tolerance,
        f'ห่าง {goal_distance:.3f} m; ถึงเป้าหมายเมื่อ ≤{.8*h.goal_tolerance:.3f} m','ภารกิจ')
    margins=[math.hypot(x,y)-probe.radius-math.hypot(x,y)*math.sin(probe._beam_width) for x,y in points]
    margin=min(margins) if margins else None
    body_clear=not any(x*x+y*y<(probe.radius+math.hypot(x,y)*math.sin(probe._beam_width))**2-1e-6 for x,y in points)
    check('footprint','ระยะรอบตัวหุ่น',None if margin is None else body_clear,
        'ยังไม่มีจุดวัด' if margin is None else f'เหลือหลังหักวงหุ่น/ระยะเผื่อ/ช่องลำแสง {margin:.3f} m; ต้อง ≥0')
    nearest=min((sector_distance((0.,0.),sector) for sector in sectors),default=None)
    rotation_radius=probe.rotation_radius(obs,now)
    check('rotation','พื้นที่หมุนไม่มี unknown ทับตัวหุ่นและระยะหยุดไหล',True if nearest is None else nearest>rotation_radius,
        'ไม่มีมุม unknown' if nearest is None else f'unknown เริ่มใกล้สุด {nearest:.3f} m; ต้อง >{rotation_radius:.3f} m (ตัวหุ่น + 0.002 m + ระยะหยุดไหล); {len(sectors)} มุม','หมุน')
    drift=rotation_radius-h.radius-.002
    rotation_space=probe.arena.contains(obs.pose.x,obs.pose.y,probe.radius+drift) and all(
        math.hypot(x,y)>=probe.radius+drift+math.hypot(x,y)*math.sin(probe._beam_width) for x,y in points)
    check('rotation_drift','จุดวัดจริงและขอบสนามพอสำหรับหยุดไหล',rotation_space,
        f'เผื่อหยุดไหล {drift:.3f} m; จุดวัดจริงยังใช้วง {probe.radius:.3f} m และระยะเผื่อลำแสง','หมุน')
    clearance=probe._clearance(0.,obs.pose,points,sectors,.13)
    state=navigator._scan_advance_state
    budget=min(.05,clearance-.025) if state is None else state[4]-state[3]-math.hypot(obs.pose.x-state[1],obs.pose.y-state[2])
    check('forward','ทางตรงก้าวตรวจ',clearance>=.045 if state is None else clearance>.025,
        f'ระยะที่ยืนยันได้ {clearance:.3f} m; ต้อง '+('≥0.045 m ก่อนเริ่ม' if state is None else '>0.025 m ระหว่างก้าว'),'ก้าวตรวจ')
    latency=max(0.,now-obs.stamp)+h.command_timeout+h.control_period
    speed=max(0.,obs.linear_speed);stop=speed*latency+speed*speed/(2*h.braking)
    space=min(budget-.005,clearance-.025)
    check('braking','พื้นที่พอสำหรับเบรกก้าวตรวจ',space>0 and stop<=space,
        f'ต้องใช้ {stop:.3f} m; มี {max(0.,space):.3f} m','ก้าวตรวจ')
    check('turn_stopped','หยุดหมุนก่อนก้าวตรง',abs(obs.angular_speed)<=.025,
        f'{obs.angular_speed:.3f} rad/s; ต้อง |ω|≤0.025','ก้าวตรวจ')
    if state is not None:
        check('advance_budget','งบระยะ/เวลาก้าวตรวจ',budget>.005 and now-state[0]<5.,
            f'เหลือ {max(0.,budget):.3f} m; ใช้เวลา {now-state[0]:.2f} s; ต้องเหลือ >0.005 m และ <5 s','ก้าวตรวจ')
    evaluated=navigator.last_checked_observation is obs
    candidates=navigator.candidate_count if evaluated else None
    check('route','ช่องทางเดินตามปกติ',None if candidates is None else candidates>0,
        'ยังไม่ประเมินการเดินตามปกติ' if candidates is None else f'{candidates} ทิศผ่านการตรวจระยะ ≥{min(.13 if sectors else .05,max(.05,goal_distance-h.goal_tolerance)):.3f} m','เดินปกติ')
    arc=navigator.last_arc_clear if evaluated else None
    check('trajectory','แนวเคลื่อนที่และระยะหยุดที่เลือก',arc,
        'ยังไม่เลือกแนวเคลื่อนที่' if arc is None else 'แนวเคลื่อนที่ผ่าน' if arc else 'แนวเคลื่อนที่ทับจุดวัด/unknown/ขอบสนาม',
        navigator.last_arc_scope if evaluated else 'เดินปกติ')
    return checks


def format_movement_conditions(checks,failed_only=False):
    lines=[]
    if failed_only:
        checks=sorted((item for item in checks if item['passed'] is not True),key=lambda item:item['passed'] is None)
    for item in checks:
        passed=item['passed']
        if failed_only and passed is True:continue
        label='ผ่าน' if passed is True else 'ไม่ผ่าน' if passed is False else 'ยังตรวจไม่ได้'
        line=f"[{label}] {item['scope']} / {item['label']}: {item['detail']}".replace('\n',' ')
        lines.append(f'{len(lines)+1}. {line}' if failed_only else line)
    return '\n'.join(lines)
