"""Setup lifecycle regressions, with no ROS nodes or hardware connections."""
from types import SimpleNamespace
from pathlib import Path
import signal
import math
import sys
import unittest
from unittest.mock import Mock, patch

from tb3_knowledge.core.localization import (LocalizationSession, _check_localization_conflicts,
    amcl_parameters,particle_support,stable_localization,localization_diagnostics)
from tb3_knowledge.core.model import Pose
from tb3_knowledge.core.ros import SetupMonitor

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import create_knowledge


class LocalizationTests(unittest.TestCase):
    def test_close_viewer_preserves_amcl_and_setup_files(self):
        process,viewer,temporary=Mock(),Mock(),Mock()
        viewer.poll.return_value=None;viewer.pid=123
        session=LocalizationSession(process,viewer,temporary)
        with patch('tb3_knowledge.core.localization.os.killpg') as kill:
            session.close_rviz();session.close_rviz()
        kill.assert_called_once_with(123,signal.SIGTERM)
        process.terminate.assert_not_called();temporary.cleanup.assert_not_called()
        self.assertIsNone(session.rviz)

    def test_report_shows_values_and_each_failed_condition(self):
        report=localization_diagnostics(dict(reason='scan_map_mismatch',map_ready=True,
            odom_age=.8,scan_age=.1,localization_age=.2,stationary=True,
            covariance=(.001,.001,.003),particle_support=.99,scan_agreement=.97,
            match_position_error=.68,match_yaw_error=math.pi/2,
            stability_samples=5,stability_span=2.,stability_position_drift=.01,stability_yaw_drift=.01,
            scan_quality=dict(separation=.01,trimmed_error=.08,endpoint_score=.4)))
        self.assertIn('[ไม่ผ่าน] อายุ odom',report)
        self.assertIn('[ผ่าน] อายุ LiDAR',report)
        self.assertIn('68.0/15 cm',report)
        self.assertIn('90.0/11.5°',report)
        self.assertIn('[ไม่ผ่าน] แยกคำตอบอื่น',report)
        self.assertIn('[ไม่ผ่าน] คะแนนปลายลำแสง',report)

    def test_global_search_params_never_assume_map_origin_is_start(self):
        params=amcl_parameters({'amcl':{'ros__parameters':{}},'other':{'unchanged':True}})
        amcl=params['amcl']['ros__parameters']
        self.assertFalse(amcl['set_initial_pose'])
        self.assertTrue(amcl['always_reset_initial_pose'])
        self.assertGreater(amcl['max_particles'],amcl['min_particles'])
        self.assertEqual(params['other'],{'unchanged':True})

    def test_separate_position_and_heading_modes_are_ambiguous(self):
        pose=Pose(.4,.4)
        for other in (Pose(1.4,.4),Pose(.4,.4,math.pi)):
            self.assertEqual(particle_support(pose,[(pose,.5),(other,.5)]),.5)
        self.assertAlmostEqual(particle_support(pose,[(pose,95),(Pose(1,1),5)]),.95)
        self.assertEqual(particle_support(pose,[]),0.)
        with self.assertRaises(ValueError):particle_support(pose,[(pose,float('nan'))])

    def test_stability_needs_several_estimates_over_time(self):
        pose=Pose(.4,.4)
        history=[(t,pose) for t in (1.,1.5,2.,2.5)]
        self.assertTrue(stable_localization(history,2.5))
        self.assertFalse(stable_localization(history[:2],1.5))
        self.assertFalse(stable_localization([(1.,Pose(.5,.4))]+history[1:],2.5))

    def test_uneven_amcl_updates_can_confirm_stability_without_relaxing_drift(self):
        pose=Pose(.4,.4)
        history=[(t,pose) for t in (1.,1.7,2.4,3.1)]
        self.assertTrue(stable_localization(history,3.1))
        self.assertFalse(stable_localization(history[:3],2.4))
        self.assertFalse(stable_localization([(1.,Pose(.44,.4))]+history[1:],3.1))
        self.assertFalse(stable_localization(history,8.))

    def test_direct_discovery_blocks_existing_localization(self):
        for graph in ('/amcl\n', '/tb3/map_server\n', '/slam_toolbox\n', '/cartographer_node\n'):
            with self.subTest(graph=graph), patch('tb3_knowledge.core.localization.subprocess.run',
                    return_value=SimpleNamespace(returncode=0,stdout=graph,stderr='')) as run:
                with self.assertRaisesRegex(ValueError,'เปิดอยู่แล้ว'):
                    _check_localization_conflicts()
                command=run.call_args.args[0]
                self.assertIn('--no-daemon',command)
                self.assertIn('--spin-time',command)

    def test_robot_bringup_is_not_a_localization_conflict(self):
        graph='/robot_state_publisher\n/turtlebot3_node\n/ld08_driver\n/diff_drive_controller\n'
        with patch('tb3_knowledge.core.localization.subprocess.run',
                return_value=SimpleNamespace(returncode=0,stdout=graph,stderr='')):
            _check_localization_conflicts()

    def test_failed_discovery_does_not_allow_another_launch(self):
        with patch('tb3_knowledge.core.localization.subprocess.run',
                return_value=SimpleNamespace(returncode=1,stdout='',stderr='discovery failed')):
            with self.assertRaisesRegex(ValueError,'discovery failed'):
                _check_localization_conflicts()

    def test_closed_viewer_reopens_without_restarting_localization(self):
        process=Mock();process.poll.return_value=None
        viewer=Mock();viewer.poll.return_value=0
        session=LocalizationSession(process,viewer,Mock())
        new_viewer=Mock();new_viewer.poll.return_value=None
        with patch('tb3_knowledge.core.localization.subprocess.Popen',return_value=new_viewer) as launch:
            session.ensure_rviz()
            session.ensure_rviz()
        launch.assert_called_once()
        self.assertEqual(launch.call_args.args[0][0],'rviz2')
        self.assertIs(session.process,process)
        self.assertIs(session.rviz,new_viewer)

    def test_dead_localization_cannot_be_hidden_by_reopening_viewer(self):
        process=Mock();process.poll.return_value=1
        with patch('tb3_knowledge.core.localization.subprocess.Popen') as launch:
            with self.assertRaises(ValueError):
                LocalizationSession(process,None,Mock()).ensure_rviz()
        launch.assert_not_called()

    def test_program1_termination_cleans_localization_and_restores_handlers(self):
        root,app=Mock(),Mock()
        handlers={sig:object() for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
        original=dict(handlers)
        def install(sig,handler):
            previous=handlers[sig];handlers[sig]=handler;return previous
        def mainloop():
            handlers[signal.SIGTERM](signal.SIGTERM,None)
        root.mainloop.side_effect=mainloop
        with patch.object(create_knowledge.tk,'Tk',return_value=root),\
             patch.object(create_knowledge,'App',return_value=app),\
             patch.object(create_knowledge.signal,'signal',side_effect=install):
            create_knowledge.main()
        root.quit.assert_called_once()
        app.stop_localization.assert_called_once()
        root.destroy.assert_called_once()
        self.assertEqual(handlers,original)

    def test_program1_exception_still_cleans_localization(self):
        root,app=Mock(),Mock()
        root.mainloop.side_effect=RuntimeError('event loop failed')
        with patch.object(create_knowledge.tk,'Tk',return_value=root),\
             patch.object(create_knowledge,'App',return_value=app),\
             patch.object(create_knowledge.signal,'signal'):
            with self.assertRaisesRegex(RuntimeError,'event loop failed'):
                create_knowledge.main()
        app.stop_localization.assert_called_once()
        root.destroy.assert_called_once()

    def test_dead_launch_still_cleans_its_viewer(self):
        app=Mock()
        app.monitor=None
        app.localization.poll.return_value=1
        localization=app.localization
        create_knowledge.App.stop_localization(app)
        localization.terminate.assert_called_once()
        self.assertIsNone(app.localization)

    def test_program1_monitor_uses_selected_map_and_automatic_search(self):
        app=Mock();app.monitor=None
        app.invalidate=Mock()
        with patch('core.ros.SetupMonitor') as factory:
            create_knowledge.App.ensure_setup_monitor(app)
            create_knowledge.App.ensure_setup_monitor(app)
        factory.assert_called_once_with(auto_localize=True,expected_map=app.slam,hardware=app.hardware)
        factory.return_value.start.assert_called_once()
        app.invalidate.assert_not_called()

    def test_retry_invalidates_previous_start_and_training(self):
        app=Mock();app.busy=False;app.simulation_mode.get.return_value=False
        app.localization.poll.return_value=None
        create_knowledge.App.retry_localization(app)
        app.monitor.request_global_localization.assert_called_once()
        app.invalidate.assert_called_once()


class AutomaticReadinessTests(unittest.TestCase):
    def prepared(self):
        monitor=SetupMonitor(auto_localize=True)
        monitor._global_request.clear();monitor._global_epoch=997.
        monitor._scan_stamp=1000.;monitor._odom_seen=100.;monitor._stationary_since=98.
        monitor._map_ready=True
        for t in (997.,998.,999.,1000.):monitor._buffer.add(t,Pose(0,0))
        pose=Pose(.4,.4)
        monitor._localization=(999.9,pose,(.001,.001,.003))
        monitor._pose_history.extend((t,pose) for t in (998.,998.5,999.,999.9))
        monitor._particles_state=(1000.,((pose,1.),))
        return monitor

    def snapshot(self,monitor):
        with patch('tb3_knowledge.core.ros.time.monotonic',return_value=100.),\
             patch('tb3_knowledge.core.ros.time.time',return_value=1000.):
            return monitor.snapshot()

    def test_only_concentrated_stable_fresh_estimate_is_ready(self):
        monitor=self.prepared()
        snapshot=self.snapshot(monitor)
        self.assertTrue(snapshot['ready'],snapshot)
        self.assertEqual(snapshot['particle_support'],1.)
        self.assertEqual(snapshot['stability_samples'],4)
        self.assertAlmostEqual(snapshot['stability_span'],1.9)
        self.assertEqual(snapshot['stability_position_drift'],0.)
        self.assertEqual(snapshot['stability_yaw_drift'],0.)

    def test_uncertain_cloud_blocks_capture_despite_small_pose_covariance(self):
        monitor=self.prepared()
        monitor._particles_state=(1000.,((Pose(.4,.4),.5),(Pose(1.4,1.4),.5)))
        self.assertEqual(self.snapshot(monitor)['reason'],'localization_ambiguous')
        with patch('tb3_knowledge.core.ros.time.monotonic',return_value=100.),\
             patch('tb3_knowledge.core.ros.time.time',return_value=1000.):
            with self.assertRaisesRegex(ValueError,'ambiguous'):monitor.capture(Mock())

    def test_missing_stale_particles_and_short_history_block(self):
        for particles,expected in ((None,'waiting_localization_particles'),
                ((997.,((Pose(.4,.4),1.),)),'waiting_localization_particles')):
            monitor=self.prepared();monitor._particles_state=particles
            self.assertEqual(self.snapshot(monitor)['reason'],expected)
        monitor=self.prepared();monitor._pose_history.clear()
        self.assertEqual(self.snapshot(monitor)['reason'],'waiting_localization_stable')

    def test_retry_rejects_old_pose_and_restarts_search(self):
        monitor=self.prepared();monitor._match_attempts=3;monitor.request_global_localization()
        self.assertEqual(monitor._match_attempts,0)
        self.assertTrue(monitor._global_request.is_set())
        self.assertIsNone(monitor._localization)
        self.assertIsNone(monitor._particles_state)
        self.assertEqual(self.snapshot(monitor)['reason'],'waiting_global_localization')
        with self.assertRaisesRegex(ValueError,'disabled'):SetupMonitor().request_global_localization()

    def test_timeout_explains_failure_but_can_recover_when_evidence_arrives(self):
        monitor=self.prepared();monitor._search_started=40.
        monitor._particles_state=None
        self.assertEqual(self.snapshot(monitor)['reason'],'automatic_localization_timeout')
        monitor._particles_state=(1000.,((Pose(.4,.4),1.),))
        self.assertTrue(self.snapshot(monitor)['ready'])

    def test_failed_scan_search_remains_blocked_then_explains_timeout(self):
        monitor=self.prepared();monitor._matching_status='scan_map_mismatch';monitor._search_started=90.
        self.assertEqual(self.snapshot(monitor)['reason'],'scan_map_mismatch')
        monitor._search_started=40.
        self.assertEqual(self.snapshot(monitor)['reason'],'automatic_localization_timeout')

    def test_map_mismatch_and_missing_scan_block_capture(self):
        monitor=self.prepared();monitor._map_error='localization_map_mismatch'
        self.assertEqual(self.snapshot(monitor)['reason'],'localization_map_mismatch')
        monitor=self.prepared();monitor._scan_stamp=998.
        self.assertEqual(self.snapshot(monitor)['reason'],'waiting_localization_scan')

    def test_only_adequate_fresh_scan_refreshes_localization_evidence(self):
        monitor=SetupMonitor(auto_localize=True)
        scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1000,nanosec=0),frame_id='base_scan'),
            ranges=[float('nan')]*120,angle_min=-math.pi,angle_increment=2*math.pi/120,range_min=0.,range_max=100.)
        with patch('tb3_knowledge.core.ros.time.time',return_value=1000.):
            monitor._setup_scan(scan);self.assertEqual(monitor._scan_stamp,0.)
            scan.ranges=[1.]*120;monitor._setup_scan(scan);self.assertEqual(monitor._scan_stamp,1000.)
            monitor._scan_stamp=0.;scan.header.stamp.sec=1001
            monitor._setup_scan(scan);self.assertEqual(monitor._scan_stamp,0.)

    def test_ros_map_pixels_must_match_selected_map_including_row_order(self):
        import numpy as np
        from tb3_knowledge.core.maps import SlamMap
        slam=SlamMap(np.array([[0,1],[2,0]]),.05,(1.,2.,0.))
        monitor=SetupMonitor(auto_localize=True,expected_map=slam)
        origin=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.),
            orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        msg=SimpleNamespace(header=SimpleNamespace(frame_id='map'),
            info=SimpleNamespace(width=2,height=2,resolution=.05,origin=origin),data=[-1,0,0,100])
        monitor._map(msg);self.assertTrue(monitor._map_ready)
        msg.data=[0,100,-1,0];monitor._map(msg)
        self.assertFalse(monitor._map_ready)
        self.assertEqual(monitor._map_error,'localization_map_mismatch')

    def test_scan_match_gate_blocks_a_concentrated_but_wrong_amcl_estimate(self):
        monitor=self.prepared();monitor.expected_map=Mock()
        monitor._matching_status='localization_ambiguous'
        self.assertEqual(self.snapshot(monitor)['reason'],'localization_ambiguous')
        monitor._matching_status='';monitor._matched_pose=Pose(1.4,1.4)
        self.assertEqual(self.snapshot(monitor)['reason'],'scan_map_mismatch')

    def test_new_positional_hint_discards_previous_search_without_creating_reference(self):
        monitor=self.prepared();generation=monitor._match_generation
        self.assertTrue(monitor.set_search_hint((.55,.7),.35))
        self.assertEqual(monitor.position_hint,(.55,.7))
        self.assertGreater(monitor._match_generation,generation)
        self.assertTrue(monitor._global_request.is_set())
        self.assertIsNone(monitor._localization)
        self.assertIsNone(monitor._matched_pose)
        self.assertFalse(monitor.set_search_hint((.55,.7),.35))
        self.assertTrue(monitor.set_search_hint(None,.35))
        with self.assertRaises(ValueError):monitor.set_search_hint((float('nan'),0.),.35)
        with self.assertRaises(ValueError):monitor.set_search_hint((.5,.7),float('nan'))
