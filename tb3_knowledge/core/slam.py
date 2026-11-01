"""Owned terminal sessions for interactive mapping and keyboard teleoperation."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import yaml


def write_cartographer_config(folder,source=None):
    """Override sensor limits locally; never edit the installed ROS package."""
    from .model import Hardware
    if source is None:
        from ament_index_python.packages import get_package_share_directory
        source=Path(get_package_share_directory('turtlebot3_cartographer'))/'config/turtlebot3_lds_2d.lua'
    content=Path(source).read_text()
    hardware=Hardware()
    for key,value in (('min_range',hardware.lidar_min),('max_range',hardware.scan_range)):
        content,count=re.subn(r'^TRAJECTORY_BUILDER_2D\.'+key+r'\s*=.*$',
            f'TRAJECTORY_BUILDER_2D.{key} = {value}',content,flags=re.MULTILINE)
        if count!=1:raise ValueError('ไม่พบค่า Cartographer ที่ต้องปรับ: '+key)
    output=Path(folder)/'tb3_aligned.lua'
    output.write_text(content)
    return output


def stop_group(pid):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                os.killpg(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.05)


class SlamSession:
    def __init__(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='tb3_slam_')
        self.terminals = []
        self.pid_files = []

    def start(self):
        from .localization import _check_localization_conflicts
        _check_localization_conflicts()
        env = dict(os.environ, TURTLEBOT3_MODEL='burger')
        try:
            config=write_cartographer_config(self.temporary.name)
            commands = (
                ('SLAM · Cartographer', ['ros2', 'launch', 'turtlebot3_cartographer', 'cartographer.launch.py',
                    'cartographer_config_dir:='+str(config.parent),'configuration_basename:='+config.name]),
                ('SLAM · Teleop', ['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard']),
            )
            for index, (title, command) in enumerate(commands):
                pid_file = Path(self.temporary.name) / f'{index}.json'
                self.pid_files.append(pid_file)
                self.terminals.append(subprocess.Popen([
                    'gnome-terminal', '--wait', '--title='+title, '--',
                    sys.executable, str(Path(__file__).resolve()), str(pid_file), *command,
                ], env=env, start_new_session=True))
            deadline = time.monotonic() + 8
            while not all(path.exists() for path in self.pid_files):
                if any(process.poll() is not None for process in self.terminals):
                    raise RuntimeError('เปิด terminal SLAM ไม่สำเร็จ ตรวจ ROS environment และ gnome-terminal')
                if time.monotonic() > deadline:
                    raise RuntimeError('หมดเวลารอ terminal SLAM')
                time.sleep(.05)
        except Exception:
            self.terminate()
            raise
        return self

    def terminate(self):
        for path in reversed(self.pid_files):
            try:
                stop_group(json.loads(path.read_text())['pid'])
            except FileNotFoundError:
                pass
        for terminal in self.terminals:
            if terminal.poll() is None:
                terminal.terminate()
                try:
                    terminal.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    terminal.kill()
                    terminal.wait()
        self.temporary.cleanup()


def save_map(prefix):
    prefix = Path(prefix).expanduser().resolve()
    if prefix.suffix.lower() in ('.yaml', '.yml', '.pgm'):
        prefix = prefix.with_suffix('')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix)],
                            capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or 'บันทึกแผนที่ไม่สำเร็จ')
    output = prefix.with_name(prefix.name+'.yaml')
    if not output.is_file():
        raise RuntimeError('map_saver ไม่ได้สร้างไฟล์ YAML; เก็บ SLAM ไว้ให้ลองใหม่')
    metadata = yaml.safe_load(output.read_text())
    image = Path(metadata['image']).expanduser()
    if not image.is_absolute():image = output.parent / image
    if not image.is_file():
        raise RuntimeError('ไม่พบไฟล์ภาพแผนที่ที่บันทึก; เก็บ SLAM ไว้ให้ลองใหม่')
    return output


def terminal_worker(pid_file, command):
    process = subprocess.Popen(command, start_new_session=True)
    def interrupted(signum, frame):
        stop_group(process.pid)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    pending = Path(pid_file).with_suffix('.tmp')
    pending.write_text(json.dumps({'pid': process.pid}))
    pending.replace(pid_file)
    try:
        return process.wait()
    finally:
        stop_group(process.pid)
        Path(pid_file).unlink(missing_ok=True)


if __name__ == '__main__':
    raise SystemExit(terminal_worker(sys.argv[1], sys.argv[2:]))
