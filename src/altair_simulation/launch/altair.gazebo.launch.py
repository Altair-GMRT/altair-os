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
    # Path to the world file
    gazebo_world_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'worlds',
        'altair_world.sdf'
    )

    # Path to the robot model file
    # already added in the altair_world.sdf file
    # robot_model_path = os.path.join(
    #     get_package_share_directory('altair_simulation'),
    #     'urdf',
    #     'robot.urdf'
    # )

    return LaunchDescription([
        # Launch Gazebo simulator
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', gazebo_world_path],
            output='screen'
        ),
        
        # Spawn a robot model
        # Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     arguments=['-name', 'my_robot', '-file', robot_model_path],
        #     output='screen'
        # )
    ])