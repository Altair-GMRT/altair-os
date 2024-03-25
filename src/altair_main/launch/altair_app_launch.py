import os
import subprocess
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


CORE_CONFIG_PATH    = os.path.join(os.getcwd(), 'src/altair_data/config/core_config.yaml')
JOINT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml')


with open(os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml'), 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']
    U2D2_PORT       = ROBOT_CONFIG['u2d2_port']
    BAUDRATE        = ROBOT_CONFIG['baudrate']


def generate_launch_description():
    
    app_controller_node = Node(
        package     = 'altair_controllers',
        executable  = 'app_controller',
        name        = f'{ROBOT_ID}_app_controller',
        parameters  = [
            {'id': ROBOT_ID},
            {'u2d2_port': U2D2_PORT},
            {'baudrate': BAUDRATE},
            {'joint_config_path': JOINT_CONFIG_PATH}
        ]
    )

    app_launcher_node = Node(
        package     = 'altair_interfaces',
        executable  = 'app_launcher',
        name        = f'{ROBOT_ID}_app_launcher'
    )

    return LaunchDescription([
        app_launcher_node
    ])