import os
import subprocess
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


with open(os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml'), 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']


def generate_launch_description():
    
    app_launcher_node = Node(
        package     = 'altair_interfaces',
        executable  = 'app_launcher',
        name        = f'{ROBOT_ID}_app_launcher'
    )

    return LaunchDescription([
        app_launcher_node
    ])