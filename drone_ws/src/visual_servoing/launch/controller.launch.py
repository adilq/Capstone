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
    
    camera_rotation_file = os.path.join(
        get_package_share_directory("visual_servoing"),
        "config",
        "rotation_camerareal_to_drone.npy"
    )
    
    bounds_file = os.path.join(
        get_package_share_directory("visual_servoing"),
        "config",
        "bounds.yaml"
    )

    camera_k_file_launch_arg = DeclareLaunchArgument(
        "camera_k", default_value=TextSubstitution(text=camera_k_file)
    )
    
    camera_rotation_file_launch_arg = DeclareLaunchArgument(
        "camera_rotation", default_value=TextSubstitution(text=camera_rotation_file)
    )
    
    bounds_file_launch_arg = DeclareLaunchArgument(
        "bounds", default_value=TextSubstitution(text=bounds_file)
    )

    return LaunchDescription([
        camera_k_file_launch_arg,
        camera_rotation_file_launch_arg,
        bounds_file_launch_arg,
        Node(
            package="visual_servoing",
            executable="controller",
            namespace="",
            name="vs_controller",
            parameters=[{
                "camera_k": LaunchConfiguration("camera_k"),
                "camera_rotation": LaunchConfiguration("camera_rotation"),
                "bounds": LaunchConfiguration("bounds")
            }]
        )
    ])
