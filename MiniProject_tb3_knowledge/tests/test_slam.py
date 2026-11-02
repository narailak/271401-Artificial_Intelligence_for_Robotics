"""Mapping commands, owned process cleanup, and save failure handling."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tb3_knowledge.core.slam import SlamSession, save_map,write_cartographer_config
from tb3_knowledge.core.model import Hardware


class SlamTests(unittest.TestCase):
    def test_cartographer_config_matches_shared_lidar_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'source.lua'
            original='include "map_builder.lua"\nTRAJECTORY_BUILDER_2D.min_range = 0.12\nTRAJECTORY_BUILDER_2D.max_range = 8.\n'
            source.write_text(original)
            output=write_cartographer_config(folder,source)
            self.assertIn(f'min_range = {Hardware().lidar_min}',output.read_text())
            self.assertIn(f'max_range = {Hardware().scan_range}',output.read_text())
            self.assertEqual(source.read_text(),original)
            source.write_text('no sensor limits')
            with self.assertRaisesRegex(ValueError,'Cartographer'):write_cartographer_config(folder,source)

    def test_launches_burger_in_two_terminals_and_stops_owned_groups(self):
        session = SlamSession()
        processes = []
        def launch(command, **kwargs):
            self.assertEqual(command[0], 'gnome-terminal')
            self.assertEqual(kwargs['env']['TURTLEBOT3_MODEL'], 'burger')
            path = Path(command[command.index('--')+3])
            path.write_text(json.dumps({'pid': 1200+len(processes)}))
            process = Mock()
            process.poll.return_value = None
            processes.append((command, process))
            return process
        with patch('tb3_knowledge.core.localization._check_localization_conflicts'), \
                patch('tb3_knowledge.core.slam.write_cartographer_config',return_value=Path(session.temporary.name)/'tb3_aligned.lua'), \
                patch('tb3_knowledge.core.slam.subprocess.Popen', side_effect=launch), \
                patch('tb3_knowledge.core.slam.stop_group') as stop:
            session.start()
            self.assertEqual(processes[0][0][-6:-2], ['ros2','launch','turtlebot3_cartographer','cartographer.launch.py'])
            self.assertEqual(processes[0][0][-1],'configuration_basename:=tb3_aligned.lua')
            self.assertTrue(processes[0][0][-2].startswith('cartographer_config_dir:='))
            self.assertEqual(processes[1][0][-4:], ['ros2','run','turtlebot3_teleop','teleop_keyboard'])
            session.terminate()
            self.assertEqual([c.args[0] for c in stop.call_args_list], [1201,1200])
            for _, process in processes:process.terminate.assert_called_once()

    def test_save_quotes_path_as_argument_and_requires_output(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'new folder'/'my arena.yaml'
            def saved(command, **kwargs):
                self.assertEqual(command, ['ros2','run','nav2_map_server','map_saver_cli','-f',str(target.with_suffix(''))])
                target.write_text('image: my arena.pgm\n')
                target.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
                return SimpleNamespace(returncode=0)
            with patch('tb3_knowledge.core.slam.subprocess.run',side_effect=saved):
                self.assertEqual(save_map(target),target)
            target.with_suffix('.pgm').unlink()
            with patch('tb3_knowledge.core.slam.subprocess.run',return_value=SimpleNamespace(returncode=0)):
                with self.assertRaisesRegex(RuntimeError,'ภาพแผนที่'):save_map(target)
            with patch('tb3_knowledge.core.slam.subprocess.run',return_value=SimpleNamespace(returncode=1,stderr='no map',stdout='')):
                with self.assertRaisesRegex(RuntimeError,'no map'):save_map(target)
            target.unlink()
            with patch('tb3_knowledge.core.slam.subprocess.run',return_value=SimpleNamespace(returncode=0)):
                with self.assertRaisesRegex(RuntimeError,'YAML'):save_map(target)


if __name__ == '__main__':unittest.main()
