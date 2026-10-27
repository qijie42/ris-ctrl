from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="airsim_image_bridge_cpp",
            executable="airsim_image_bridge",
            name="airsim_image_bridge",
            output="screen",
            parameters=[{
                "airsim_host": "127.0.0.1",
                "vehicle_name": "PX4",
                "left_camera": "CameraImage",
                "right_camera": "CameraRight",
                "depth_camera": "CameraDepth",
                "camera_rate": 30.0,
            }],
        )
    ])
