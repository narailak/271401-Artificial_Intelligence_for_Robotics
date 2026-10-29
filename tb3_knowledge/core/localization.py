"""Setup-only map server/AMCL launch. Never creates a velocity publisher."""
import os
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import yaml

STABILITY_WINDOW=4.0


def localization_diagnostics(snapshot):
    """Display measured values beside the unchanged capture thresholds."""
    lines=[]
    def check(label,passed,detail=''):
        state='รอข้อมูล' if passed is None else 'ผ่าน' if passed else 'ไม่ผ่าน'
        lines.append(f'[{state}] {label}: {detail}')
    check('แมพตรงกัน',snapshot.get('map_ready'),snapshot.get('reason',''))
    for key,label,limit in (('odom_age','อายุ odom',.4),('scan_age','อายุ LiDAR',.6),('localization_age','อายุ AMCL',5.)):
        value=snapshot.get(key)
        check(label,None if value is None else -.06<=value<=limit,
            'ยังไม่มีข้อมูล' if value is None else f'{value:.3f} s (รับ -0.060 ถึง {limit:.3f} s)')
    check('หุ่นอยู่นิ่ง',snapshot.get('stationary'),'ต้องนิ่งก่อนรับ Start')
    cov=snapshot.get('covariance')
    check('ความไม่แน่นอน AMCL',None if cov is None else cov[0]<=.0025 and cov[1]<=.0025 and cov[2]<=.0077,
        'ยังไม่มี pose' if cov is None else f'variance x/y/มุม = {cov[0]:.5f}/{cov[1]:.5f}/{cov[2]:.5f} (จำกัด .0025/.0025/.0077)')
    support=snapshot.get('particle_support')
    check('กลุ่มสมมติฐาน',None if support is None else support>=.95,'รอ AMCL' if support is None else f'{support:.1%} / ต้อง ≥95%')
    agreement=snapshot.get('scan_agreement')
    if agreement is not None:check('ลำแสงตรงแมพ',agreement>=.90,f'{agreement:.1%} / ต้อง ≥90%')
    quality=snapshot.get('scan_quality') or {}
    for key,label,limit,minimum in (('separation','แยกคำตอบอื่น',.04,True),
            ('trimmed_error','ระยะคลาดเฉลี่ยหลังตัด outlier',.07,False),('endpoint_score','คะแนนปลายลำแสง',.65,True)):
        value=quality.get(key)
        if value is not None:check(label,value>=limit if minimum else value<=limit,
            f'{value:.3f} / ต้อง {"≥" if minimum else "≤"}{limit:.3f}')
    position,yaw=snapshot.get('match_position_error'),snapshot.get('match_yaw_error')
    check('AMCL ตรงกับตำแหน่งตั้งต้น',None if position is None or yaw is None else position<=.15 and yaw<=.2,
        'รอผลเทียบตำแหน่ง' if position is None or yaw is None else f'{position*100:.1f}/15 cm · {math.degrees(yaw):.1f}/11.5°')
    count,span=snapshot.get('stability_samples',0),snapshot.get('stability_span',0.)
    drift,angle=snapshot.get('stability_position_drift',0.),snapshot.get('stability_yaw_drift',0.)
    check('ความนิ่งของผลประมาณ',count>=4 and span>=1.5 and drift<=.025 and angle<=.07,
        f'{count}/4 ค่า · {span:.1f}/1.5 s · แกว่ง {drift*100:.1f}/2.5 cm, {math.degrees(angle):.1f}/4°')
    return '\n'.join(lines)

def particle_support(pose, particles):
    """Weight near one pose; separate position OR heading modes stay ambiguous.

    This measures the filter's concentration, not ground-truth accuracy. A
    visually symmetric scene can still require an operator's initial estimate.
    """
    total=near=0.
    for candidate,weight in particles:
        if not math.isfinite(weight) or weight<0:raise ValueError('invalid_particle_weights')
        total+=weight
        angle=(candidate.yaw-pose.yaw+math.pi)%(2*math.pi)-math.pi
        if math.hypot(candidate.x-pose.x,candidate.y-pose.y)<=.12 and abs(angle)<=.20:
            near+=weight
    return near/total if total>0 else 0.


def stable_localization(history, stamp):
    """Require four consistent estimates spanning at least 1.5 seconds.

    AMCL no-motion updates can arrive unevenly. A four-second window keeps
    the same sample count, minimum duration and spatial/angular tolerances.
    Freshness is checked separately before accepting the pose.
    """
    recent=[(t,p) for t,p in history if stamp-STABILITY_WINDOW<=t<=stamp]
    if len(recent)<4 or recent[-1][0]-recent[0][0]<1.5:return False
    last=recent[-1][1]
    return all(math.hypot(p.x-last.x,p.y-last.y)<=.025 and
        abs((p.yaw-last.yaw+math.pi)%(2*math.pi)-math.pi)<=.07 for _,p in recent)


def amcl_parameters(params):
    """Setup-only settings for a small LDS-02 arena and global initialization."""
    amcl=params['amcl']['ros__parameters']
    amcl.update(base_frame_id='base_footprint',odom_frame_id='odom',global_frame_id='map',scan_topic='/scan',
        update_min_d=.02,update_min_a=.03,laser_min_range=.16,laser_max_range=8.,transform_tolerance=.2,
        min_particles=2000,max_particles=8000,max_beams=120,sigma_hit=.06,z_hit=.95,z_rand=.05,
        set_initial_pose=False,always_reset_initial_pose=True,resample_interval=1)
    return params


class LocalizationSession:
    def __init__(self, process, rviz, temporary):
        self.process,self.rviz,self.temporary=process,rviz,temporary

    def poll(self):
        return self.process.poll()

    def close_rviz(self):
        """Close only the owned viewer; capture/export still need AMCL."""
        if self.rviz is not None and self.rviz.poll() is None:
            try:os.killpg(self.rviz.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:self.rviz.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(self.rviz.pid,signal.SIGKILL);self.rviz.wait()
        self.rviz=None

    def ensure_rviz(self):
        """Reopen a closed viewer while keeping the same AMCL session."""
        if self.process.poll() is not None:
            raise ValueError('Localization หยุดแล้ว เปิด session ใหม่ก่อน')
        if self.rviz is None or self.rviz.poll() is not None:
            config=Path(__file__).resolve().parents[1]/'setup.rviz'
            self.rviz=subprocess.Popen(['rviz2','-d',str(config)],start_new_session=True)

    def terminate(self):
        for process in (self.process,self.rviz):
            if process and process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:os.killpg(process.pid,signal.SIGTERM)
                    except ProcessLookupError:pass
                    try:process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait()
        self.temporary.cleanup()


def _check_localization_conflicts():
    # Inspect DDS directly: a stale ros2 daemon can report an empty graph even
    # while AMCL is running, which otherwise permits duplicate map->odom owners.
    result=subprocess.run(['ros2','node','list','--no-daemon','--spin-time','3'],
        capture_output=True,text=True,timeout=10)
    if result.returncode:
        raise ValueError(result.stderr.strip() or 'Cannot inspect ROS graph')
    conflicts=[]
    for name in result.stdout.splitlines():
        leaf=name.strip().rsplit('/',1)[-1].lower()
        if (leaf in ('amcl','map_server','lifecycle_manager_localization')
                or any(token in leaf for token in ('cartographer','slam_toolbox'))):
            conflicts.append(name.strip())
    if conflicts:
        raise ValueError('มี SLAM/localization เปิดอยู่แล้ว: '+', '.join(sorted(set(conflicts)))+
            ' · หยุดชุดเดิมก่อนเปิดใหม่ เพื่อไม่ให้ map/TF ซ้ำกัน')


def start_localization(yaml_path,image_path=None):
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup=Path(get_package_share_directory('nav2_bringup'))
        get_package_share_directory('rviz2')
    except (ImportError,LookupError) as exc:
        raise ValueError('Source ROS Jazzy first; nav2_bringup and rviz2 are required for live localization') from exc
    # Full-map setup is opt-in. Never leave two localization publishers fighting
    # over map->odom, and never start a Nav2 navigation controller here.
    _check_localization_conflicts()
    temporary=tempfile.TemporaryDirectory(prefix='tb3_setup_')
    folder=Path(temporary.name)
    source=Path(yaml_path).expanduser().resolve()
    map_data=yaml.safe_load(source.read_text())
    original=Path(map_data['image']).expanduser()
    image=Path(image_path).expanduser().resolve() if image_path else original if original.is_absolute() else source.parent/original
    if not image.is_file():temporary.cleanup();raise FileNotFoundError(str(image))
    map_data['image']=str(image)
    map_file=folder/'setup-map.yaml';map_file.write_text(yaml.safe_dump(map_data))
    params=amcl_parameters(yaml.safe_load((bringup/'params/nav2_params.yaml').read_text()))
    params_file=folder/'localization.yaml';params_file.write_text(yaml.safe_dump(params))
    process=rviz=None
    try:
        process=subprocess.Popen(['ros2','launch','nav2_bringup','localization_launch.py',
            'map:='+str(map_file),'params_file:='+str(params_file),'use_sim_time:=false','autostart:=true',
            'use_composition:=False'],start_new_session=True)
        session=LocalizationSession(process,rviz,temporary)
        session.ensure_rviz()
        return session
    except Exception:
        LocalizationSession(process,rviz,temporary).terminate()
        raise
