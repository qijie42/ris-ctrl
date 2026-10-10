from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ned_odom_topic',
            default_value='/airsim_node/PX4/odom_local_ned',
            description='Input NED odometry topic'
        ),
        DeclareLaunchArgument(
            'ned_pointcloud_topic',
            default_value='/airsim_node/PX4/lidar/LidarSensor1',
            description='Input NED pointcloud topic'
        ),
        DeclareLaunchArgument(
            'enu_odom_topic',
            default_value='/enu/odom',
            description='Output ENU odometry topic'
        ),
        DeclareLaunchArgument(
            'global_pointcloud_topic',
            default_value='/global/pointcloud',
            description='Output global pointcloud topic'
        ),

        # 启动 NED 到 ENU 转换节点
        Node(
            package='airsim_bridge',
            executable='transform_node.py',
            name='ned_to_enu_converter',
            output='screen',
            parameters=[{
                'ned_odom_topic': LaunchConfiguration('ned_odom_topic'),
                'ned_pointcloud_topic': LaunchConfiguration('ned_pointcloud_topic'),
                'enu_odom_topic': LaunchConfiguration('enu_odom_topic'),
                'global_pointcloud_topic': LaunchConfiguration('global_pointcloud_topic'),
            }]
        ),
    ])
