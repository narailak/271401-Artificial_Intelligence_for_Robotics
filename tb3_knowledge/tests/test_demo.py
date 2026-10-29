"""Offline demo runs without map files or ROS and leaves Knowledge unchanged."""
import math
import unittest

from tb3_knowledge.core.demo import DemoWorld, run_demo
from tb3_knowledge.core.model import Arena, Knowledge, Pose


class DemoTests(unittest.TestCase):
    def test_circle_is_generated_and_start_goal_are_clear(self):
        knowledge = Knowledge(Arena(2.5, 2.5), Pose(.4, .4, math.pi / 4), (2.1, 2.1))
        world = DemoWorld(knowledge)
        self.assertEqual(len(world.obstacles), 1)
        self.assertTrue(world.free(knowledge.start.x, knowledge.start.y, knowledge.hardware.safe_radius))
        self.assertTrue(world.free(*knowledge.goal, knowledge.hardware.safe_radius))
        self.assertFalse(world.segment_free(knowledge.start, knowledge.goal, knowledge.hardware.radius))
        self.assertTrue(all(math.isfinite(v) and v > 0 for v in world.scan(knowledge.start)))

    def test_demo_uses_knowledge_without_mutation_or_source_map(self):
        knowledge = Knowledge(Arena(.8, .8), Pose(.2, .4), (.6, .4))
        before = knowledge.document()
        result = run_demo(knowledge)
        self.assertTrue(result["demo"])
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["demo_obstacles"], [])
        self.assertGreater(len(result["trace"]), 2)
        self.assertEqual(knowledge.document(), before)


if __name__ == "__main__":
    unittest.main()
