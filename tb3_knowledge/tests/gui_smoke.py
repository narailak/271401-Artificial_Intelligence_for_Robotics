"""Real-display Program 1 import/training/export and mission-change regression."""
import hashlib
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import create_knowledge
from create_knowledge import App
from core.model import Knowledge


def main():
    root=tk.Tk();app=App(root)
    callback_errors=[]
    root.report_callback_exception=lambda exc,value,tb:callback_errors.append((exc.__name__,str(value)))
    try:
        widget=app.hardware_button
        while widget is not None and widget is not app.setup_tab:widget=widget.master
        assert widget is app.setup_tab,'Hardware/clearance must be configured on the mission page'
        assert tuple(app.candidate_box['values'])==tuple(str(i) for i in range(1,9))
        source=Path.home()/'tb3_maps/arena.yaml'
        inputs=[source,source.with_suffix('.pgm')]
        if not all(p.exists() for p in inputs):
            print('GUI smoke skipped: actual arena fixture not available');return
        hashes=[hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs]
        app.load_map(source);root.update()
        assert app.boundary_detection is not None,'Actual arena walls should be detected on load'
        assert 'ตรวจแนวผนังสี่ด้านแล้ว' in app.map_info.get()
        detected=app.boundary_detection.arena
        assert abs(float(app.arena_vars['width'].get())-detected.width)<.0001
        app.detect_button.invoke();root.update()
        assert app.boundary_detection is not None
        start,goal=app.validate_mission()
        assert math.dist((start.x,start.y),goal)>=.5, 'Suggested Goal must differ from Start'
        # Live Start comes only from ROS; there is no rough-position UI.
        app.simulation_mode.set(False);app.mode_changed();app.click_mode.set('start')
        mx,my=app.arena.local_to_map(start.x,start.y)
        u,v=app.slam.map_to_pixel(mx,my);ox,oy,scale=app.image_transform
        previous=app.mission()[0]
        app.click(SimpleNamespace(x=ox+(u+.5)*scale,y=oy+(v+.5)*scale));root.update()
        assert app.mission()[0]==previous
        assert app.captured_start is None and app.reference.mode=='simulation'
        assert not hasattr(app,'hint_select_button')
        assert not app.canvas.find_withtag('localization_hint')
        assert 'ไม่ผ่าน' in app.action_feedback.get()
        app.simulation_mode.set(True);app.mode_changed()
        app.set_pose(start,goal);app.click_mode.set('goal');root.update()
        assert app.canvas.find_withtag('start_heading')
        assert 'rclpy' not in sys.modules,'Changing mission mode must not start ROS'
        app.mission_button.invoke();root.update()
        assert app.mission_state=='valid' and 'ผ่าน' in app.mission_feedback.get()
        valid_feedback=app.mission_feedback.get()
        assert app.tabs.select()==str(app.setup_tab),'Successful verification should leave its feedback visible'
        original_width=app.arena_vars['width'].get()
        app.arena_vars['width'].set(str(float(original_width)+.01))
        assert app.arena_fields_dirty and app.mission_state=='pending'
        for value in (app.arena_vars['width'].get(),original_width):
            app.arena_vars['width'].set(value)
            try:app.validate_mission()
            except ValueError as exc:assert 'กดใช้ค่ากรอบ' in str(exc),str(exc)
            else:raise AssertionError('Unapplied arena fields allowed mission verification')
        app.arena_button.invoke();root.update()
        assert not app.arena_fields_dirty and app.mission_state=='pending'
        app.mission_button.invoke();root.update()
        assert app.mission_state=='valid','Applied frame should allow a fresh mission check'
        old_goal=app.pose_vars['gx'].get()
        app.pose_vars['gx'].set('0')
        assert app.mission_state=='pending','Editing a validated mission must clear its success state'
        with patch.object(create_knowledge.messagebox,'showerror') as showerror:
            app.mission_button.invoke();root.update()
            assert showerror.called,'Invalid Start–Goal should explain the failure'
        assert app.mission_state=='invalid' and app.mission_feedback.get()!=valid_feedback
        app.pose_vars['gx'].set(old_goal)
        assert app.mission_state=='pending','Restoring an old value still requires another verification'
        for key in ('x','gx'):
            original=app.pose_vars[key].get()
            for value in ('','-','nan','inf','1e309'):
                app.pose_vars[key].set(value);root.update()
                try:app.mission()
                except ValueError:pass
                else:raise AssertionError(f'Invalid mission coordinate accepted: {key}={value}')
                assert app.mission_state=='pending'
            app.pose_vars[key].set(original)
        assert not callback_errors,callback_errors
        app.mission_button.invoke();root.update()
        assert app.mission_state=='valid'
        def check_footprints():
            scale=app.image_transform[2]/app.slam.resolution
            for name in ('start','goal'):
                body=app.canvas.coords(f'{name}_body');margin=app.canvas.coords(f'{name}_margin')
                assert len(body)==4 and len(margin)==4
                assert math.isclose(body[2]-body[0],2*app.hardware.radius*scale,abs_tol=1e-6)
                assert math.isclose(body[3]-body[1],2*app.hardware.radius*scale,abs_tol=1e-6)
                assert math.isclose(margin[2]-margin[0],2*app.hardware.safe_radius*scale,abs_tol=1e-6)
        check_footprints()
        original_hardware=app.hardware
        app.hardware=replace(app.hardware,radius=.13,safety_margin=.05);app.draw()
        check_footprints()
        app.hardware=original_hardware;app.invalidate();app.draw()
        assert app.mission_state=='pending','Hardware changes invalidate earlier mission checks'
        app.mission_button.invoke();root.update()
        assert app.mission_state=='valid'
        app.candidates.set('2');app.build_knowledge()
        deadline=time.monotonic()+100
        while app.busy and time.monotonic()<deadline:
            root.update();time.sleep(.01)
        assert not app.busy and app.policy is not None, app.status.get()
        assert app.trained_context is not None
        for used,button in ((False,app.baseline_button),(True,app.knowledge_button)):
            button.invoke()
            assert app.preview_result is None,'Starting another run must clear the previous result'
            deadline=time.monotonic()+60
            while app.busy and time.monotonic()<deadline:
                root.update();time.sleep(.01)
            assert not app.busy and app.preview_result is not None,app.status.get()
            assert app.preview_uses_knowledge is used
            assert app.preview_context==app.trained_context
            result=app.preview_result;summary=app.simulation_summary.get()
            assert f"เวลา {result['time']:.2f} วินาที" in summary,summary
            assert f"ระยะทาง {result['distance']:.3f} เมตร" in summary,summary
            assert ('ใช้ Knowledge' if used else 'ค่าเริ่มต้น') in summary,summary
            assert len(app.canvas.find_withtag('simulation_path'))<=1,'Only the latest path should be drawn'
            for key,expected in {'time':f"{result['time']:.2f}",'distance':f"{result['distance']:.3f}",
                    'goal_error':f"{result['goal_error']:.3f}",'collisions':str(result['collisions'])}.items():
                assert app.simulation_values[key].get()==expected,(key,app.simulation_values[key].get(),expected)
                assert app.metric_widgets[key].winfo_ismapped(),f'{key} metric should remain visible'
        assert 'ใต้แผนที่' in app.status.get()
        with tempfile.TemporaryDirectory() as tmp:
            create_knowledge.OUTPUT=Path(tmp)
            output=Path(tmp)/'knowledge.json'
            exported=app.export_to(output)
            reloaded=Knowledge.load(output)
            assert exported.fingerprint==reloaded.fingerprint
            assert reloaded.reference.mode=='simulation'
            assert 'obstacles' not in json.dumps(reloaded.document())
            assert 'rclpy' not in sys.modules
            assert app.frozen
            try:app.export_to(create_knowledge.OUTPUT/'training_report.json')
            except ValueError as exc:assert 'สงวน' in str(exc),str(exc)
            else:raise AssertionError('Knowledge/report filename collision allowed')
            old_goal=app.pose_vars['gx'].get()
            app.pose_vars['gx'].set(str(float(old_goal)-.03))
            assert app.mission_state=='pending','A frozen export must not preserve a stale success badge'
            assert app.preview_result is None,'Edited mission must clear the previous simulation result'
            assert 'ยังไม่มีผลซิม' in app.simulation_summary.get()
            try:app.export_to(output)
            except ValueError as exc:
                assert 'เปลี่ยนหลังฝึก' in str(exc), str(exc)
            else:raise AssertionError('Changed mission reused prior training')
            app.pose_vars['gx'].set(old_goal)
        assert hashes==[hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs]
        for geometry,columns in (('1360x880',4),('1000x680',2)):
            root.geometry(geometry)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                root.update()
                if app.result_columns==columns:break
                time.sleep(.01)
            assert app.result_columns==columns,(geometry,app.result_columns)
            assert app.canvas.winfo_width()>250 and app.canvas.winfo_height()>200
            assert app.results_panel.winfo_ismapped()
            frames=list(app.metric_widgets.values())
            assert len(frames)==4
            for index,frame in enumerate(frames):
                assert frame.winfo_ismapped()
                assert frame.winfo_x()>=0 and frame.winfo_y()>=0
                assert frame.winfo_x()+frame.winfo_width()<=app.results_panel.winfo_width()
                for other in frames[index+1:]:
                    overlap_x=min(frame.winfo_x()+frame.winfo_width(),other.winfo_x()+other.winfo_width())-max(frame.winfo_x(),other.winfo_x())
                    overlap_y=min(frame.winfo_y()+frame.winfo_height(),other.winfo_y()+other.winfo_height())-max(frame.winfo_y(),other.winfo_y())
                    assert overlap_x<=0 or overlap_y<=0,'Simulation metrics must not overlap'
            check_footprints()
        assert not callback_errors,callback_errors
        print('GUI passed: mission success/failure/stale feedback, finite input, responsive latest-run metrics, scaled robot footprints, full training, frozen export, no ROS, originals unchanged')
    finally:
        root.destroy()


if __name__=='__main__':main()
