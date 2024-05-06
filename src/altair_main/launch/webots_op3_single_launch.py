import os
import yaml
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


CORE_CONFIG_PATH    = os.path.join(os.getcwd(), 'src/altair_data/config/core_config.yaml')
JOINT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml')
PKG_SHARE_PATH      = get_package_share_directory('altair_simulations')
ROBOT_DESC_PATH     = os.path.join(PKG_SHARE_PATH, 'resource', 'op3_single.urdf')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']
    MASTER_CLOCK    = ROBOT_CONFIG['master_clock']


with open(JOINT_CONFIG_PATH, 'r') as file:
    JOINT_CONFIG    = yaml.safe_load(file)
    DXL_BAUDRATE    = JOINT_CONFIG['dxl_baudrate']
    DXL_U2D2_PORT   = JOINT_CONFIG['dxl_u2d2_port']
    DXL_NUM         = JOINT_CONFIG['dxl_num']
    DXL_ID          = JOINT_CONFIG['dxl_id']
    DXL_TYPE        = JOINT_CONFIG['dxl_type']
    JOINT_NAME      = JOINT_CONFIG['joint_name']


def generate_launch_description():
    
    webots = WebotsLauncher(
        world = os.path.join(PKG_SHARE_PATH, 'worlds', 'op3_single_world.wbt')
    )

    robotis_op3_driver = WebotsController(
        robot_name  = 'robotis_op3',
        parameters  = [
            {'robot_description': ROBOT_DESC_PATH},
        ]
    )

    return LaunchDescription([
        webots,
        robotis_op3_driver,
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action   = webots,
                on_exit         = [launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])