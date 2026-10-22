"""Launch all ris-ctrl bridges: PX4, LiDAR, and Camera (C++ version)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    ego_position_topic = LaunchConfiguration('ego_position_topic')

    launch_args = [
        DeclareLaunchArgument(
            'ego_position_topic',
            default_value='/drone_0_planning/pos_cmd',
            description='Ego-Planner PositionCommand input topic',
        ),
    ]

    # 获取Python脚本路径（只用于激光雷达）
    ris_ctrl_src = os.path.expanduser('~/1_ws/src/ris-ctrl/src/airsim_bridge/src')
    
    return LaunchDescription(
        launch_args + [
            # PX4 Bridge节点
            Node(
                package='px4_bridge',
                executable='bridge',
                name='px4_bridge',
                output='screen',
            ),
            
            # Odom Bridge节点
            Node(
                package='px4_bridge',
                executable='odom_bridge',
                name='odom_to_vehicle_odometry_bridge',
                output='screen',
            ),

            # Ego-Planner PositionCommand -> px4_bridge RealtimeControl
            Node(
                package='px4_bridge',
                executable='ego_position_command_bridge',
                name='ego_position_command_bridge',
                output='screen',
                parameters=[{
                    'input_topic': ego_position_topic,
                    'output_topic': '/px4_bridge/in/realtime_control',
                }],
            ),
            
            # 激光雷达数据桥接（Python版本）
            ExecuteProcess(
                cmd=['python3', os.path.join(ris_ctrl_src, 'getLidarData.py')],
                name='lidar_bridge',
                output='screen',
            ),
            
            # 相机数据桥接（C++版本）
            Node(
                package='airsim_image_bridge_cpp',
                executable='airsim_image_bridge',
                name='airsim_image_bridge',
                output='screen',
            ),
        ]
    )
