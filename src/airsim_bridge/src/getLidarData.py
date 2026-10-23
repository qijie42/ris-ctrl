import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import struct
import time


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_publisher")
        self.client = airsim.MultirotorClient("127.0.0.1")
        self.client.confirmConnection()
        self.odom_translation = None
        self.odom_rotation_matrix = None
        self.local_pointcloud_pub = self.create_publisher(PointCloud2, "/lidar/points_body", 10)
        self.global_pointcloud_pub = self.create_publisher(PointCloud2, "/lidar/points_world", 10)
        self.odom_pub = self.create_publisher(Odometry, "/drone260/odom", 10)
        self.timer = self.create_timer(0.05, self.publish_pointcloud)
        self.timer = self.create_timer(0.01, self.publish_odom)
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ])

    def transform_points_vectorized(self, points_array):
        rotated_points = (self.odom_rotation_matrix @ points_array.T).T
        return rotated_points + self.odom_translation

    def ned_to_enu_position(self, ned_pos):
        """AirSim NED (m) -> ROS ENU (m): swap x/y, negate z"""
        return [ned_pos[1], ned_pos[0], -ned_pos[2]]

    def ned_to_enu_quaternion(self, qx, qy, qz, qw):
        """NED to ENU: 90deg rotation around Z-axis"""
        s = 0.7071067811865476
        return [s * (qy + qx), s * (qy - qx), s * (qw + qz), s * (qw - qz)]

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"
        try:
            pose = self.client.simGetVehiclePose(vehicle_name="PX4")
            ned_pos = [pose.position.x_val, pose.position.y_val, pose.position.z_val]
            ned_quat = [pose.orientation.x_val, pose.orientation.y_val, pose.orientation.z_val, pose.orientation.w_val]
            enu_pos = self.ned_to_enu_position(ned_pos)
            enu_quat = self.ned_to_enu_quaternion(*ned_quat)
            odom_msg.pose.pose.position.x = enu_pos[0]
            odom_msg.pose.pose.position.y = enu_pos[1]
            odom_msg.pose.pose.position.z = enu_pos[2]
            odom_msg.pose.pose.orientation.x = enu_quat[0]
            odom_msg.pose.pose.orientation.y = enu_quat[1]
            odom_msg.pose.pose.orientation.z = enu_quat[2]
            odom_msg.pose.pose.orientation.w = enu_quat[3]
            kinematics = self.client.simGetGroundTruthKinematics(vehicle_name="PX4")
            ned_vel = [kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val]
            enu_vel = self.ned_to_enu_position(ned_vel)
            odom_msg.twist.twist.linear.x = enu_vel[0]
            odom_msg.twist.twist.linear.y = enu_vel[1]
            odom_msg.twist.twist.linear.z = enu_vel[2]
            self.odom_pub.publish(odom_msg)
            self.odom_rotation_matrix = self.quaternion_to_rotation_matrix(*enu_quat)
            self.odom_translation = np.array(enu_pos)
        except Exception as e:
            self.get_logger().error(f"Failed to get pose: {e}")

    def publish_pointcloud(self):
        local_points = PointCloud2()
        global_points = PointCloud2()
        data = self.client.getLidarData(lidar_name="Lidar", vehicle_name="PX4")
        if len(data.point_cloud) < 3:
            return
        local_points.header.stamp = self.get_clock().now().to_msg()
        local_points.header.frame_id = "base_link"
        local_points.height = 1
        local_points.width = len(data.point_cloud) // 3
        local_points.fields = self.fields
        local_points.is_bigendian = False
        local_points.point_step = 12
        local_points.row_step = local_points.point_step * local_points.width
        local_points.is_dense = True
        global_points.header.stamp = local_points.header.stamp
        global_points.header.frame_id = "world"
        global_points.height = local_points.height
        global_points.width = local_points.width
        global_points.fields = local_points.fields
        global_points.is_bigendian = local_points.is_bigendian
        global_points.point_step = local_points.point_step
        global_points.row_step = local_points.row_step
        global_points.is_dense = local_points.is_dense
        body_points = np.array(data.point_cloud).reshape(-1, 3)
        body_points_flu = body_points.copy()
        body_points_flu[:, 1] = -body_points[:, 1]
        body_points_flu[:, 2] = -body_points[:, 2]
        data_local = body_points_flu.flatten().tolist()
        format_str = f"{len(data_local)}f"
        local_points.data = struct.pack(format_str, *data_local)
        self.local_pointcloud_pub.publish(local_points)
        if self.odom_rotation_matrix is not None and self.odom_translation is not None:
            transformed_points = self.transform_points_vectorized(body_points_flu)
            world_points = transformed_points.flatten().tolist()
            global_points.data = struct.pack(format_str, *world_points)
            self.global_pointcloud_pub.publish(global_points)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
