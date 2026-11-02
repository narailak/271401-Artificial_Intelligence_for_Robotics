"""Notebook command watchdog. No ROS dependency until explicitly started.

This process protects against controller failure while ROS transport works. It
cannot deliver a stop after notebook power loss or a Wi-Fi partition.
"""
from __future__ import annotations

import argparse
import math
import signal
import threading
import time


def motor_message_type(subscriptions,requested='auto'):
    """Resolve the driver's actual input type without creating a publisher."""
    known={'geometry_msgs/msg/TwistStamped':'stamped','geometry_msgs/msg/Twist':'twist'}
    if requested not in ('auto','stamped','twist'):raise ValueError('invalid_motor_output_type')
    kinds={endpoint.topic_type for endpoint in subscriptions}
    if not kinds:return None
    if len(kinds)!=1 or not kinds.issubset(known):
        raise ValueError('motor_topic_type_mismatch: subscribers expose incompatible types '+', '.join(sorted(kinds)))
    detected=known[next(iter(kinds))]
    if requested!='auto' and requested!=detected:
        raise ValueError(f'motor_topic_type_mismatch: driver requires {detected}; selected {requested}')
    return detected


def validate_velocity(v: float, w: float, *, max_speed: float, max_angular: float):
    if not all(math.isfinite(n) for n in (v, w)) or v < 0:
        raise ValueError("nonfinite or reverse command")
    return min(v, max_speed), max(-max_angular, min(max_angular, w))


def make_guard_node(*, input_topic, output_topic, output_type="stamped",
                    max_speed=.08, max_angular=.8, timeout=.35,
                    base_frame="base_footprint", name="knowledge_guard", context=None):
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    if output_type not in ("stamped", "twist"):
        raise ValueError("output_type must be stamped or twist")
    if not all(math.isfinite(x) and x > 0 for x in (max_speed, max_angular, timeout)):
        raise ValueError("invalid guard limits")

    class Guard(Node):
        def __init__(self):
            super().__init__(name, context=context)
            self.input_topic = self.resolve_topic_name(input_topic)
            self.output_topic = self.resolve_topic_name(output_topic)
            if self.input_topic == self.output_topic:
                raise ValueError("requested and motor topics must differ")
            self.kind = "geometry_msgs/msg/TwistStamped" if output_type == "stamped" else "geometry_msgs/msg/Twist"
            self.pub = self.create_publisher(TwistStamped if output_type == "stamped" else Twist, self.output_topic, 10)
            self.status = self.create_publisher(String, self.input_topic + "/guard_status", 10)
            self.sub = self.create_subscription(TwistStamped, self.input_topic, self.receive, 10)
            self.stop_service = self.create_service(Trigger, "~/stop", self.stop_request)
            self.last_receive = 0.
            self.last_stamp = 0.
            self.velocity = (0., 0.)
            self.latched = ""
            self.reason = "waiting_command"
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.025, self.tick, clock=self.steady)

        def stop_request(self, request, response):
            self.latched = "operator_stop"
            self.zero()
            response.success = True
            response.message = "Stopped; restart guard and controller for next mission"
            return response

        def receive(self, msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.latched:
                return
            if msg.header.frame_id.lstrip("/") != base_frame.lstrip("/"):
                self.latched = "command_frame_mismatch"
                self.zero()
                return
            if not math.isfinite(stamp) or stamp <= self.last_stamp or now - stamp > timeout or stamp - now > .06:
                # Replays do not refresh the monotonic watchdog.
                self.reason = "stale_command"
                self.last_receive = 0.
                self.zero()
                return
            t = msg.twist
            if any(not math.isfinite(n) or abs(n) > 1e-9 for n in (t.linear.y, t.linear.z, t.angular.x, t.angular.y)):
                self.latched = "invalid_command_axes"
                self.zero()
                return
            try:
                self.velocity = validate_velocity(t.linear.x, t.angular.z, max_speed=max_speed, max_angular=max_angular)
            except ValueError:
                self.latched = "invalid_command"
                self.zero()
                return
            self.last_stamp = stamp
            self.last_receive = time.monotonic()

        def graph_reason(self):
            publishers = self.get_publishers_info_by_topic(self.output_topic)
            foreign = [p for p in publishers if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
            # Duplicate node names still produce separate publisher endpoints.
            if foreign or len(publishers) > 1:
                return "competing_motor_publisher", True
            wanted = self.get_publishers_info_by_topic(self.input_topic)
            if len(wanted) > 1:
                return "competing_requested_publisher", True
            drivers = self.get_subscriptions_info_by_topic(self.output_topic)
            if any(d.topic_type != self.kind for d in drivers):
                return "motor_topic_type_mismatch", True
            if not any(d.topic_type == self.kind for d in drivers):
                return "waiting_motor_subscriber", False
            return "", False

        def tick(self):
            issue, latch = self.graph_reason()
            if latch:
                self.latched = issue
            if self.latched:
                self.reason = self.latched
                self.zero()
            elif issue:
                self.reason = issue
                self.zero()
            elif time.monotonic() - self.last_receive > timeout:
                self.reason = "command_timeout"
                self.zero()
            else:
                self.reason = "forwarding"
                self.send(*self.velocity)
            m = String()
            m.data = self.reason
            self.status.publish(m)

        def send(self, v, w):
            m = TwistStamped() if output_type == "stamped" else Twist()
            t = m.twist if output_type == "stamped" else m
            if output_type == "stamped":
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = base_frame
            t.linear.x, t.angular.z = float(v), float(w)
            self.pub.publish(m)

        def zero(self):
            self.send(0., 0.)

        def cleanup(self):
            # A flush/spin can execute tick after this call; forwarding must
            # remain impossible even when the previous command is still fresh.
            self.latched = self.latched or "shutdown"
            self.velocity = (0., 0.)
            self.last_receive = 0.
            self.zero()

    return Guard()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Managed notebook-side velocity guard")
    parser.add_argument("--input-topic", default="/knowledge/cmd_vel")
    parser.add_argument("--output-topic", default="/cmd_vel")
    parser.add_argument("--output-type", choices=("stamped", "twist"), default="stamped")
    parser.add_argument("--max-speed", type=float, default=.08)
    parser.add_argument("--max-angular", type=float, default=.8)
    parser.add_argument("--timeout", type=float, default=.35)
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stop_requested = threading.Event()
    def interrupted(signum, frame):
        stop_requested.set()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, interrupted)
    try:
        node = make_guard_node(input_topic=args.input_topic, output_topic=args.output_topic,
                               output_type=args.output_type, max_speed=args.max_speed,
                               max_angular=args.max_angular, timeout=args.timeout,
                               base_frame=args.base_frame)
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        # Repeated Ctrl+C/SIGTERM must not interrupt final zero delivery.
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            node.cleanup()
            # Keep the publisher alive long enough for the motor driver to
            # receive zero before DDS removes the endpoint.
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                try:
                    node.zero()
                    rclpy.spin_once(node, timeout_sec=.025)
                except Exception:
                    break
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
