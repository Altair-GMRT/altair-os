import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # this name has to match the robot name in the Xacro file
    robotXacroName='robotis_op3'
    
    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    namePackage = 'op3_description'
    
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'urdf/robotis_op3.urdf.xacro'
    worldFileRelativePath = 'worlds/empty.world'
    
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)


    # this is the absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    
    # this is the launch file from the gazebo_ros package
    # gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py'))
    
    


    # this is the launch description   
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch)
    rvizLaunch=ExecuteProcess(cmd=['rviz2', os.path.join(get_package_share_directory(namePackage), 'rviz', 'op3.rviz')], output='screen')

    # gazeboLaunch=ExecuteProcess(
    #     cmd=['gazebo', '-u', os.path.join(get_package_share_directory(namePackage), 'worlds', 'empty.world'), "--verbose",
    #          "-s", "libgazebo_ros_factory.so",
    #          "-s", "libgazebo_ros_init.so", "--ros-args" 
    #          ],
    #     output='screen'
    # )

    # ------------------------- Node ------------------
    # Create a gazebo_ros Node 
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
                          arguments=['-topic','robot_description','-entity', robotXacroName],output='screen')


    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True, 'get_model_state': True}] 
    )

    # Joint State Publisher Node
    nodeJointRobotStatePublisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    nodeJointRobotStatePublisherGUI = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    node_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"]
    )
    node_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["effort_controller", "-c", "/controller_manager"]
    )
    # node_joint_broad = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=["joint_broad", "-c", "/controller_manager"]
    # )

    
    # ------------------------------------------------------------
    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()
     
    # we add gazeboLaunch 
    launchDescriptionObject.add_action(rvizLaunch)
    
    # we add the two nodes
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(nodeJointRobotStatePublisher)
    launchDescriptionObject.add_action(node_joint_trajectory_controller)
    launchDescriptionObject.add_action(node_effort_controller)
    # launchDescriptionObject.add_action(node_joint_broad)
    launchDescriptionObject.add_action(nodeJointRobotStatePublisherGUI)

    
    return launchDescriptionObject