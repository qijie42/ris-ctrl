"""Isolated adapter regression tests; never start the PX4 control node."""
import math
import time
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand
from px4_bridge_msgs.msg import RealtimeControl
from px4_bridge.ego_position_command_bridge import EgoPositionCommandBridge


class AdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[
            '--ros-args', '-p', 'input_topic:=/ego_adapter_test/input',
            '-p', 'output_topic:=/ego_adapter_test/output',
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = EgoPositionCommandBridge()
        self.real_publisher = self.node.publisher
        self.node.publisher = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def command(self):
        cmd = PositionCommand()
        cmd.header.frame_id = 'world'
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.position.x, cmd.position.y, cmd.position.z = 1.0, 2.0, 3.0
        cmd.velocity.x, cmd.velocity.y, cmd.velocity.z = 0.1, 0.2, 0.3
        cmd.acceleration.x = 0.4
        cmd.yaw, cmd.yaw_dot = 0.5, 0.05
        return cmd

    def test_maps_all_fields(self):
        self.node._on_position_command(self.command())
        out = self.node.publisher.publish.call_args.args[0]
        self.assertTrue(out.enable and out.has_command and out.use_position)
        self.assertTrue(out.use_velocity and out.use_acceleration and out.use_yaw_rate)
        self.assertFalse(out.use_attitude or out.use_jerk)
        self.assertEqual((out.x, out.y, out.z), (1.0, 2.0, 3.0))
        for value, expected in ((out.vx, .1), (out.vy, .2), (out.vz, .3),
                                (out.ax, .4), (out.yaw, .5), (out.yaw_rate, .05)):
            self.assertAlmostEqual(value, expected, places=6)

    def test_rejects_nan_inf_and_float32_overflow_without_crash(self):
        for value in (math.nan, math.inf, -math.inf, 1e100):
            with self.subTest(value=value):
                cmd = self.command()
                cmd.position.x = value
                self.node._on_position_command(cmd)
        self.node.publisher.publish.assert_not_called()

    def test_rejects_old_zero_future_timestamps(self):
        for shift in (-2, 2):
            cmd = self.command()
            cmd.header.stamp.sec += shift
            self.node._on_position_command(cmd)
        cmd = self.command()
        cmd.header.stamp.sec = cmd.header.stamp.nanosec = 0
        self.node._on_position_command(cmd)
        self.node.publisher.publish.assert_not_called()

    def test_rejects_wrong_frame(self):
        cmd = self.command()
        cmd.header.frame_id = 'base_link'
        self.node._on_position_command(cmd)
        self.node.publisher.publish.assert_not_called()

    def test_ignores_non_ready_states(self):
        for flag in (0, 3, 4, 5, 6, 7):
            cmd = self.command()
            cmd.trajectory_flag = flag
            self.node._on_position_command(cmd)
        self.node.publisher.publish.assert_not_called()

    def test_real_ros_message_roundtrip(self):
        self.node.publisher = self.real_publisher
        peer = Node('ego_adapter_test_peer')
        received = []
        pub = peer.create_publisher(PositionCommand, '/ego_adapter_test/input', 1)
        sub = peer.create_subscription(
            RealtimeControl, '/ego_adapter_test/output', received.append, 1)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(peer)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and not received:
                if pub.get_subscription_count() and self.real_publisher.get_subscription_count():
                    pub.publish(self.command())
                executor.spin_once(timeout_sec=0.03)
            self.assertTrue(received, 'No converted ROS message received')
            self.assertAlmostEqual(received[-1].x, 1.0)
            self.assertAlmostEqual(received[-1].yaw_rate, .05, places=6)
        finally:
            executor.shutdown()
            peer.destroy_subscription(sub)
            peer.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
