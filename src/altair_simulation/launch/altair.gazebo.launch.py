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
    spawn_world = ExecuteProcess(
            cmd=['gz', 'sim', '-v','-r', gazebo_world_path],
            output='screen'
        )
    
    topic_brige_path = os.path.join(
    get_package_share_directory('altair_simulation'),
    'config',
    'bridge.yaml'
    )
    topic_brige = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={topic_brige_path}',
        ],
        output='screen',
    )
    camera_bridge = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['gazebo/camera/image_raw'],
    output='screen',
)

    # Path to the robot model file
    # already added in the altair_world.sdf file
    robot_model_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robot.urdf'
    )
    with open(robot_model_path, 'r') as infp:
        robot_desc = infp.read()

    robot_urdf_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robot.urdf'
    )
    robot_xacro_path = os.path.join(
        get_package_share_directory('altair_simulation'),
        'urdf',
        'robotis_op3.urdf.xacro'
    )

    convert_urdf_xacro = ExecuteProcess(
        cmd=['xacro', robot_xacro_path, '-o', robot_urdf_path],
        output='screen'
    )

    spawn_robot = ExecuteProcess(
        cmd=["ros2 launch ros_gz_sim gz_spawn_model.launch.py file:=$(ros2 pkg prefix --share altair_simulation)/urdf/robot.urdf name:=robot"],
        output='screen'
    )

    # joint_state_publisher_gui
    joint_state_publisher_gui = Node(
     package='joint_state_publisher_gui',
     executable='joint_state_publisher_gui',
     name='joint_state_publisher_gui',
     arguments=[robot_model_path],
     output=['screen']
    )
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[
        {'use_sim_time': True},
        {'robot_description': robot_desc},
    ]
)

    return LaunchDescription([
        # Launch Gazebo simulator
        spawn_world,
        topic_brige,
        camera_bridge,
        # convert_urdf_xacro,
        # spawn_robot,
        joint_state_publisher_gui,
        robot_state_publisher

    ])