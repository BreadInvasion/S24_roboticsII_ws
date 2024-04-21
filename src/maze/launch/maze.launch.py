import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    lidar = Node(
            package = 'maze',
            executable = 'lidar')

    controller = Node(
            package = 'maze',
            executable = 'cont')
    pos_controller = Node(
            package = 'maze',
            executable = 'position_cont')

    return LaunchDescription([lidar,controller,pos_controller])
