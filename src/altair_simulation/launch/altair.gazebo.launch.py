import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    
    return launchDescriptionObject
