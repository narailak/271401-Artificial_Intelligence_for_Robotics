"""Program 2 file picker, process launch/stop and simulation-drive exclusion."""
from pathlib import Path
import json
import io
import sys
import tempfile
import time
import tkinter as tk
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.model import Arena,Knowledge,Pose
from run_knowledge import RunnerApp,main,parser


def pump(root,condition,timeout=12):
    deadline=time.monotonic()+timeout
    while not condition() and time.monotonic()<deadline:
        root.update();time.sleep(.01)
    root.update()
    assert condition(),'Runner UI operation timed out'


def run():
    with patch('run_knowledge.launch_gui') as launch:
        assert main([])==0
        launch.assert_called_once()
    root=tk.Tk();app=RunnerApp(root,parser().parse_args([]))
    try:
        root.update()
        assert app.output_type.get()=='auto','Driver type should be detected from subscribers by default'
        assert not app.rviz.get(),'RViz should be optional and disabled by default'
        assert str(app.demo_button['state'])=='disabled'
        with tempfile.TemporaryDirectory(prefix='knowledge picker ') as tmp:
            path=Path(tmp)/'selected knowledge.json'
            Knowledge(Arena(2.5,2.5),Pose(.4,.4,.7),(2.,2.)).save(path)
            with patch('tkinter.filedialog.askopenfilename',return_value=str(path)):
                app.choose_button.invoke()
            assert app.selected_path==path.resolve()
            assert app.knowledge is not None
            assert str(app.demo_button['state'])=='normal'
            assert str(app.drive_button['state'])=='disabled'
            assert app.process is None, 'File selection must not start a process'
            root.update()
            assert app.live_view.canvas.winfo_width()>500
            packet=dict(fingerprint=app.knowledge.fingerprint,pose=dict(x=.4,y=.4,yaw=.7),
                points=[[.3,.2]],path=[[.4,.4],[.6,.6]],requested=[0.,0.],
                elapsed=2.,distance=.1,estimated_goal_error=2.,scan_age=.1,
                drive=False,reason='navigating',mode='GOAL',guard='dry_run',
                scan_total_beams=256,scan_valid_beams=239,scan_missing_beams=17,
                scan_coverage_percent=100*239/256,scan_missing_percent=100*17/256,
                scan_message_age=.05,unknown_scan_sectors=12,filled_scan_gaps=5)
            packet['movement_conditions']=[dict(code='rotation',label='พื้นที่หมุน',passed=False,
                detail='unknown 0.128 m; ต้อง >0.164 m',scope='หมุน'),
                dict(code='route',label='ช่องทางเดิน',passed=None,detail='รอข้อมูล',scope='เดินปกติ')]
            app.events.put('TB3_DATA: '+json.dumps(packet)+'\n');app.poll()
            packet['pose']=dict(x=.41,y=.4,yaw=.7)
            app.events.put('TB3_DATA: '+json.dumps(packet)+'\n');app.poll()
            assert app.live_view.values['elapsed'].get()=='2.00'
            assert app.live_view.values['coverage'].get()=='93.4'
            assert app.live_view.values['missing'].get()=='6.6'
            assert app.live_view.values['counts'].get()=='239 / 256'
            assert app.live_view.values['unknown'].get()=='12'
            condition_text=app.conditions_text.get('1.0','end')
            assert '[ไม่ผ่าน] หมุน / พื้นที่หมุน' in condition_text
            assert '[ยังตรวจไม่ได้] เดินปกติ / ช่องทางเดิน' in condition_text
            assert '0.128' in condition_text and '0.164' in condition_text
            assert app.live_view.coverage_gate.get()=='เกณฑ์สแกน ≥ 90%: ผ่าน'
            blockers=[dict(code='raw_coverage',label='ความครบสแกนล่าสุด',passed=False,
                detail='86.0%; ต้อง ≥90%',scope='ระบบ'),
                dict(code='odom_stamp',label='อายุ odometry',passed=False,
                detail='0.600 s; ต้อง ≤0.500 s',scope='ระบบ')]
            app.live_view.update_data(dict(packet,scan_coverage_percent=86.,scan_missing_percent=14.,
                scan_total_beams=100,scan_valid_beams=86,scan_missing_beams=14,movement_conditions=blockers))
            assert app.live_view.coverage_gate.get()=='เกณฑ์สแกน ≥ 90%: ไม่ผ่าน'
            lines=app.live_view.blocker_text.get('1.0','end-1c').splitlines()
            assert len(lines)==2
            assert lines[0].startswith('1.') and '86.0%' in lines[0] and '≥90%' in lines[0]
            assert lines[1].startswith('2.') and '0.600' in lines[1] and '0.500' in lines[1]
            assert str(app.live_view.blocker_text.cget('wrap'))=='none'
            app.live_view.update_data(packet)
            assert '86.0%' not in app.live_view.blocker_text.get('1.0','end')
            for tag in ('arena','robot','lidar','planned_path','trail'):
                assert app.live_view.canvas.find_withtag(tag),tag
            assert 'TB3_DATA:' not in app.console.get('1.0','end')
            app.live_view.update_data(dict(packet,stop_latched=True,explanation='หยุด: เวลา LiDAR ไม่ถูกต้อง',
                sensor_fault=dict(stream='scan',age=.773,reason='stale_or_future_scan')))
            assert '0.773' in app.live_view.fault_notice.get()
            app.live_view.update_data(packet)
            assert not app.live_view.fault_notice.get()
            previous=app.live_view.data
            app.live_view.update_data(dict(packet,requested=[]))
            assert app.live_view.data is previous
            app.live_view.update_data(dict(packet,scan_coverage_percent=101.))
            assert app.live_view.data is previous
            child=Mock();child.poll.return_value=None;child.stdout=io.StringIO('')
            with patch('run_knowledge.subprocess.Popen',return_value=child) as launch:
                app.start('dry')
            arguments=launch.call_args.args[0]
            assert '--ui-stream' in arguments
            assert '--rviz' not in arguments and '--drive' not in arguments
            assert app.live_view.data is None
            assert '0.128' not in app.live_view.blocker_text.get('1.0','end')
            app.process=None;app.sync()
            with patch('tkinter.filedialog.askopenfilename',return_value=''):
                app.choose_file()
            assert app.selected_path==path.resolve()
            bad=Path(tmp)/'bad.json';bad.write_text('{}')
            with patch('tkinter.filedialog.askopenfilename',return_value=str(bad)),patch('tkinter.messagebox.showerror') as error:
                app.choose_file();error.assert_called_once()
            assert app.selected_path==path.resolve(),'Invalid choice changed the valid selection'
            # Runtime blockers must replace the launch message while keeping
            # stop available; a delayed status must not hide a pending stop.
            fake_process=Mock()
            fake_process.poll.return_value=None
            app.process=fake_process;app.stopping=False;app.sync()
            app.status.set('เริ่มโหมดเดินจริง หุ่นจะเดินเมื่อเซนเซอร์และ guard พร้อม')
            blocked='หยุด: พบ Localization/AMCL ใช้แมพเดิม ปิดจาก Program 1 แล้วหยุดและเริ่ม Program 2 ใหม่'
            app.events.put('TB3_STATUS: '+blocked+'\n')
            app.poll()
            assert app.status.get()==blocked
            assert blocked in app.console.get('1.0','end')
            assert str(app.stop_button['state'])=='normal'
            app.stop_button.invoke()
            fake_process.terminate.assert_called_once()
            stopping_status=app.status.get()
            app.events.put('TB3_STATUS: '+blocked+'\n')
            app.poll()
            assert app.status.get()==stopping_status
            assert str(app.stop_button['state'])=='disabled'
            app.process=None;app.stopping=False;app.sync()
            # A failing preflight must retain the actual cause on the main
            # status label after the process exits, not only its exit code.
            failed=Mock();failed.poll.return_value=2
            app.process=failed
            cause='motor_topic_type_mismatch: driver requires stamped; selected twist'
            app.events.put('Error: '+cause+'\n');app.poll()
            assert app.process is None and cause in app.status.get()
            app.demo_button.invoke()
            assert app.process is not None
            process=app.process
            assert str(path.resolve()) in process.args
            assert '--demo' in process.args and '--drive' not in process.args
            pump(root,lambda:'"demo": true' in app.console.get('1.0','end'))
            app.stop_button.invoke()
            pump(root,lambda:app.process is None)
            assert process.poll() is not None
            assert app.status.get()=='หยุดแล้ว'
            assert 'rclpy' not in sys.modules
        print('Runner UI passed: file picker, runtime blocker status, stop, selected-file demo and no real drive.')
    finally:
        if app.process and app.process.poll() is None:
            app.process.terminate();app.process.wait(timeout=5)
        root.destroy()


if __name__=='__main__':run()
