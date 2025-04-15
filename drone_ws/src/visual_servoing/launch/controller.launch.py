import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    camera_k_file = os.path.join(
        get_package_share_directory("visual_servoing"),
        "config",
        "cameraK.npy"
    )

    camera_k_file_launch_arg = DeclareLaunchArgument(
        "camera_k", default_value=TextSubstitution(text=camera_k_file)
    )

    return LaunchDescription([
        camera_k_file_launch_arg,
        Node(
            package="visual_servoing",
            executable="controller",
            namespace="",
            name="vs_controller",
            parameters=[{
                "camera_k": LaunchConfiguration("camera_k")
            }]
        )
    ])
