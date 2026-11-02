from pathlib import Path
import sys,tkinter as tk
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path('tb3_knowledge').resolve()))
from create_knowledge import App
root=tk.Tk();app=App(root)
try:
 assert app.hardware.radius==.124 and app.hardware.safety_margin==.03
 with patch.object(app,'suggest_mission',side_effect=ValueError('ไม่มีคู่จุดที่เชื่อมกัน')):
  app.load_map('arena.yaml');root.update()
  assert app.slam is not None and app.mission_state=='invalid'
  assert 'เลือก Start–Goal เอง' in app.status.get()
 app.load_map('arena.yaml');root.update()
 assert len(app.tabs.tabs())==3
 assert app.tabs.tab(app.slam_tab,'text')=='SLAM map'
 widget=app.stop_localization_button
 while widget is not None and widget is not app.train_tab:widget=widget.master
 assert widget is app.train_tab
 labels=[str(w.cget('text')) for w in app.map_button.master.winfo_children() if 'text' in w.keys()]
 assert labels.index('ขั้นที่ 1 · เลือกแผนที่')<labels.index('ขั้นที่ 2 · Localize และรับ Start')<labels.index('ขั้นที่ 3 · ตั้งค่ากรอบสนาม / หุ่น / Start–Goal')
 # Exercise the save worker without opening ROS or commanding the robot.
 def immediate_thread(*,target,**kwargs):return Mock(start=target)
 session=Mock();app.slam_session=session
 with patch('tkinter.filedialog.asksaveasfilename',return_value='/tmp/test-slam-new-map.yaml'), \
      patch('create_knowledge.threading.Thread',side_effect=immediate_thread), \
      patch('core.slam.save_map',side_effect=RuntimeError('save failed')):
  app.save_slam_map();app.poll()
 assert app.slam_session is session
 session.terminate.assert_not_called()
 assert not app.busy and 'save failed' in app.slam_status.get()
 with patch('tkinter.filedialog.asksaveasfilename',return_value='/tmp/test-slam-new-map.yaml'), \
      patch('create_knowledge.threading.Thread',side_effect=immediate_thread), \
      patch('core.slam.save_map',return_value=Path('arena.yaml')):
  app.save_slam_map();app.poll()
 session.terminate.assert_called_once()
 assert app.slam_session is None and app.tabs.select()==str(app.setup_tab)
 assert not hasattr(app,'hint_select_button')
 app.run_action('ล้างผลซิม',app.clear_trace)
 assert app.action_feedback.get().startswith('ผ่าน:')
 with patch('tkinter.filedialog.askopenfilename',return_value=''):
  app.map_button.invoke()
 assert app.action_feedback.get().startswith('ยกเลิก:')
 app.simulation_mode.set(False);app.mode_changed()
 assert all(str(e['state'])=='readonly' for e in app.start_entries)
 app.run_action('ทดสอบไม่ผ่าน',lambda:False)
 assert app.action_feedback.get().startswith('ไม่ผ่าน:')
 app.busy=True;callback=Mock();app.run_action('ล้างผลซิม',callback)
 callback.assert_not_called();assert 'ไม่ผ่าน:' in app.action_feedback.get()
 app.busy=False
 snapshot=dict(ready=True,reason='ready',blocking_reason='ready',map_ready=True,odom_age=.05,scan_age=.1,
  localization_age=.1,stationary=True,covariance=(.001,.001,.003),particle_support=1.,scan_agreement=.97,
  match_position_error=.02,match_yaw_error=.01,stability_samples=4,stability_span=1.8,
  stability_position_drift=.01,stability_yaw_drift=.01)
 app.monitor=Mock();app.monitor.snapshot.return_value=snapshot
 app.localization=Mock()
 app.poll();app.poll()
 app.localization.close_rviz.assert_called_once()
 app.localization.terminate.assert_not_called()
 assert 'ผ่าน: Localize' in app.action_feedback.get()
 assert '[ผ่าน] อายุ odom' in app.localization_report.get()
 snapshot.update(ready=False,reason='scan_map_mismatch',blocking_reason='scan_map_mismatch',match_position_error=.68)
 app.poll()
 assert '[ไม่ผ่าน] AMCL ตรง' in app.localization_report.get()
 assert 'scan_map_mismatch' in app.localization_report.get()
 print('Program 1 actions passed: no rough UI, success/cancel/failure, readonly Start, diagnostics and RViz-only auto-close.')
finally:app.stop_localization();app.stop_slam();root.destroy()
