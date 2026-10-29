"""Read-only motor protocol discovery before any guard publisher is created."""
from types import SimpleNamespace
import unittest

from tb3_knowledge.core.guard import motor_message_type


def endpoints(*types):
    return [SimpleNamespace(topic_type='geometry_msgs/msg/'+name) for name in types]


class MotorDiscoveryTests(unittest.TestCase):
    def test_detects_both_driver_protocols(self):
        self.assertEqual(motor_message_type(endpoints('TwistStamped')),'stamped')
        self.assertEqual(motor_message_type(endpoints('Twist')),'twist')
        self.assertEqual(motor_message_type(endpoints('TwistStamped','TwistStamped')),'stamped')

    def test_waits_for_driver_discovery(self):
        self.assertIsNone(motor_message_type([]))

    def test_explicit_wrong_choice_and_mixed_subscribers_fail_closed(self):
        for choices,requested in ((('TwistStamped',),'twist'),(('Twist','TwistStamped'),'auto'),(('Pose',),'auto')):
            with self.assertRaisesRegex(ValueError,'motor_topic_type_mismatch'):
                motor_message_type(endpoints(*choices),requested)
        self.assertEqual(motor_message_type(endpoints('Twist'),'twist'),'twist')
