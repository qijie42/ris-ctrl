#!/usr/bin/env python3
"""NED 到 ENU 坐标转换节点"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np


class NEDtoENUConverter(Node):
    def __init__(self):
        super().__init__('ned_to_enu_converter')
        
        # 参数
        self.declare_parameter('ned_odom_topic', '/airsim_node/PX4/odom_local_ned')
        self.declare_parameter('ned_pointcloud_topic', '/airsim_node/PX4/lidar/LidarSensor1')
        self.declare_parameter('enu_odom_topic', '/enu/odom')
        self.declare_parameter('global_pointcloud_topic', '/global/pointcloud')
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('sensor_frame', 'sensor_link')
        
        ned_odom = self.get_parameter('ned_odom_topic').value
        ned_pc = self.get_parameter('ned_pointcloud_topic').value
        enu_odom = self.get_parameter('enu_odom_topic').value
        global_pc = self.get_parameter('global_pointcloud_topic').value
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, ned_odom, self.odom_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, ned_pc, self.pointcloud_callback, 10)
        
        # 发布
        self.odom_pub = self.create_publisher(Odometry, enu_odom, 10)
        self.pc_pub = self.create_publisher(PointCloud2, global_pc, 10)
        
        self.get_logger().info(f'NED→ENU 转换节点已启动')
        self.get_logger().info(f'  订阅: {ned_odom}, {ned_pc}')
        self.get_logger().info(f'  发布: {enu_odom}, {global_pc}')
    
    def ned_to_enu_position(self, x_ned, y_ned, z_ned):
        """NED 坐标转换到 ENU"""
        return y_ned, x_ned, -z_ned
    
    def ned_to_enu_quaternion(self, qw, qx, qy, qz):
        """NED 四元数转换到 ENU (FLU)"""
        # NED(FRD) -> ENU(FLU)
        return qw, qy, qx, -qz
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        enu_msg = Odometry()
        enu_msg.header = msg.header
        enu_msg.header.frame_id = 'world'
        enu_msg.child_frame_id = 'base_link'
        
        # 位置转换
        x_enu, y_enu, z_enu = self.ned_to_enu_position(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        enu_msg.pose.pose.position.x = x_enu
        enu_msg.pose.pose.position.y = y_enu
        enu_msg.pose.pose.position.z = z_enu
        
        # 姿态转换
        qw_enu, qx_enu, qy_enu, qz_enu = self.ned_to_enu_quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        )
        enu_msg.pose.pose.orientation.w = qw_enu
        enu_msg.pose.pose.orientation.x = qx_enu
        enu_msg.pose.pose.orientation.y = qy_enu
        enu_msg.pose.pose.orientation.z = qz_enu
        
        # 速度转换
        vx_enu, vy_enu, vz_enu = self.ned_to_enu_position(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )
        enu_msg.twist.twist.linear.x = vx_enu
        enu_msg.twist.twist.linear.y = vy_enu
        enu_msg.twist.twist.linear.z = vz_enu
        
        # 角速度转换
        wx_enu, wy_enu, wz_enu = self.ned_to_enu_position(
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        )
        enu_msg.twist.twist.angular.x = wx_enu
        enu_msg.twist.twist.angular.y = wy_enu
        enu_msg.twist.twist.angular.z = wz_enu
        
        # 协方差
        enu_msg.pose.covariance = msg.pose.covariance
        enu_msg.twist.covariance = msg.twist.covariance
        
        self.odom_pub.publish(enu_msg)
    
    def pointcloud_callback(self, msg):
        """处理点云消息"""
        try:
            # 读取点云
            points_list = []
            for point in pc2.read_points(msg, skip_nans=True):
                x_ned, y_ned, z_ned = point[:3]
                x_enu, y_enu, z_enu = self.ned_to_enu_position(x_ned, y_ned, z_ned)
                points_list.append([x_enu, y_enu, z_enu])
            
            if not points_list:
                return
            
            # 创建新点云
            enu_msg = pc2.create_cloud_xyz32(msg.header, points_list)
            enu_msg.header.frame_id = 'world'
            
            self.pc_pub.publish(enu_msg)
        except Exception as e:
            self.get_logger().error(f'点云转换失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NEDtoENUConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
