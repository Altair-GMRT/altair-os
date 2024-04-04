import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


CORE_CONFIG_PATH    = os.path.join(os.getcwd(), 'src/altair_data/config/core_config.yaml')
JOINT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/joint_config.yaml')
ROBOT_CONFIG_PATH   = os.path.join(os.getcwd(), 'src/altair_data/config/robot_config.yaml')


with open(ROBOT_CONFIG_PATH, 'r') as file:
    ROBOT_CONFIG    = yaml.safe_load(file)
    ROBOT_ID        = ROBOT_CONFIG['id']


def generate_launch_description():

    rclnodetest1_node = Node(
        package     = 'altair_py_tests',
        executable  = 'custom_msg_test',
        name        = f'{ROBOT_ID}_custom_msg_test1',
        parameters  = [
            {'name': 'Dhonan'},
            {'age': 32},
            {'score': 3.21},
            {'pub_topic': f'{ROBOT_ID}/topic1'},
            {'sub_topic': f'{ROBOT_ID}/topic2'}
        ]
    )

    rclnodetest2_node = Node(
        package     = 'altair_py_tests',
        executable  = 'custom_msg_test',
        name        = f'{ROBOT_ID}_custom_msg_test2',
        parameters  = [
            {'name': 'Nabil'},
            {'age': 21},
            {'score': 4.00},
            {'pub_topic': f'{ROBOT_ID}/topic2'},
            {'sub_topic': f'{ROBOT_ID}/topic1'}
        ]
    )

    return LaunchDescription([
        rclnodetest1_node,
        rclnodetest2_node
    ])