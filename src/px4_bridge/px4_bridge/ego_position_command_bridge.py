"""Convert Ego-Planner PositionCommand messages to realtime PX4 commands."""

from __future__ import annotations

import math
import time

import rclpy
from px4_bridge_msgs.msg import RealtimeControl
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node


class EgoPositionCommandBridge(Node):
    """Adapt Ego-Planner's ENU position command to the PX4 bridge interface."""

    def __init__(self) -> None:
        super().__init__("ego_position_command_bridge")
        self.declare_parameter("input_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("output_topic", "/px4_bridge/in/realtime_control")
        self.declare_parameter("publish_velocity", True)
        self.declare_parameter("publish_acceleration", True)
        self.declare_parameter("expected_frame", "world")
        self.declare_parameter("max_command_age_s", 0.3)
        self.declare_parameter("future_tolerance_s", 0.05)
        self.expected_frame = str(self.get_parameter("expected_frame").value)
        self.max_command_age_s = float(self.get_parameter("max_command_age_s").value)
        self.future_tolerance_s = float(self.get_parameter("future_tolerance_s").value)
        if (not math.isfinite(self.max_command_age_s) or self.max_command_age_s <= 0
                or not math.isfinite(self.future_tolerance_s)
                or self.future_tolerance_s < 0):
            raise ValueError("Invalid command age limits")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publish_velocity = bool(self.get_parameter("publish_velocity").value)
        self.publish_acceleration = bool(
            self.get_parameter("publish_acceleration").value
        )

        self.publisher = self.create_publisher(RealtimeControl, output_topic, 1)
        self.subscription = self.create_subscription(
            PositionCommand, input_topic, self._on_position_command, 1
        )
        self.get_logger().info(
            f"Forwarding Ego-Planner commands: {input_topic} -> {output_topic}"
        )

    @staticmethod
    def _finite(value: float, name: str) -> float:
        value = float(value)
        if not math.isfinite(value) or abs(value) > 3.4028234663852886e38:
            raise ValueError(f"{name} is not a finite float32")
        return value

    def _on_position_command(self, command: PositionCommand) -> None:
        if command.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            return

        try:
            if command.header.frame_id != self.expected_frame:
                raise ValueError(f"expected frame {self.expected_frame!r}")
            stamp_ns = (command.header.stamp.sec * 1_000_000_000
                        + command.header.stamp.nanosec)
            age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if stamp_ns <= 0 or not -self.future_tolerance_s <= age_s <= self.max_command_age_s:
                raise ValueError(f"stale or invalid command timestamp (age={age_s:.3f}s)")
            msg = RealtimeControl()
            msg.enable = True
            msg.has_command = True
            msg.use_position = True
            msg.use_velocity = self.publish_velocity
            msg.use_acceleration = self.publish_acceleration
            msg.use_attitude = False
            msg.use_jerk = False
            msg.use_yaw_rate = True
            msg.x = self._finite(command.position.x, "position.x")
            msg.y = self._finite(command.position.y, "position.y")
            msg.z = self._finite(command.position.z, "position.z")
            msg.vx = self._finite(command.velocity.x, "velocity.x")
            msg.vy = self._finite(command.velocity.y, "velocity.y")
            msg.vz = self._finite(command.velocity.z, "velocity.z")
            msg.ax = self._finite(command.acceleration.x, "acceleration.x")
            msg.ay = self._finite(command.acceleration.y, "acceleration.y")
            msg.az = self._finite(command.acceleration.z, "acceleration.z")
            msg.yaw = self._finite(command.yaw, "yaw")
            msg.yaw_rate = self._finite(command.yaw_dot, "yaw_dot")
            msg.timestamp_ms = int(time.time() * 1000)
        except ValueError as exc:
            self.get_logger().warning(
                f"Ignoring invalid PositionCommand: {exc}", throttle_duration_sec=2.0
            )
            return

        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EgoPositionCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
