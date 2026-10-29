from pathlib import Path
import sys,tkinter as tk
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path('tb3_knowledge').resolve()))
from create_knowledge import App
root=tk.Tk();app=App(root)
try:
 app.load_map('arena.yaml');root.update()
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
finally:app.stop_localization();root.destroy()
