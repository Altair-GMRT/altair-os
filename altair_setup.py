import os
import subprocess
import socket
import yaml



OS_PATH                 = '/altair-os'
ALTAIR_CONTROLLERS_PATH = os.path.join(OS_PATH, 'src/altair_controllers')
ALTAIR_INTERFACES_PATH  = os.path.join(OS_PATH, 'src/altair_interfaces')
ALTAIR_DATA_PATH        = os.path.join(OS_PATH, 'src/altair_data')
ALTAIR_MOTION_PATH      = os.path.join(OS_PATH, 'src/altair_motion')
ALTAIR_VISION_PATH      = os.path.join(OS_PATH, 'src/altair_vision')
ALTAIR_MAIN_PATH        = os.path.join(OS_PATH, 'src/altair_main')

HOST_OS_PATH            = os.getcwd()
HOST_CORE_CONFIG_PATH   = os.path.join(HOST_OS_PATH, 'src/altair_data/core_config.yaml')
HOST_ROBOT_CONFIG_PATH  = os.path.join(HOST_OS_PATH, 'src/altair_data/robot_config.yaml')



def main(args=None) -> None:
    print('----------------------------------')
    print('         ALTAIR-OS SETUP          ')
    print('----------------------------------')


    # Set the ID or namespace
    print('[1/3] Insert the ID or namespace for this machine: ')
    id = input('>> ')


    # Get the local IP
    print('[2/3] Insert the local network')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0)
    
    try:
        sock.connect(('10.254.254.254', 1))
        ip_addr = sock.getsockname()[0]
    
    except Exception:
        ip_addr = '127.0.0.1'
    
    finally:
        sock.close()
    
    ip = ip_addr
    print(f'>> {ip}')

    # Config file setup
    print('[3/3] Setting up config files...')
    
    core_config_yaml = {
        'os_path': OS_PATH,
        'altair_controllers_path': ALTAIR_CONTROLLERS_PATH,
        'altair_interfaces_path': ALTAIR_INTERFACES_PATH,
        'altair_data_path': ALTAIR_DATA_PATH,
        'altair_motion_path': ALTAIR_MOTION_PATH,
        'altair_vision_path': ALTAIR_VISION_PATH,
        'altair_main_path': ALTAIR_MAIN_PATH
    }
    with open(HOST_CORE_CONFIG_PATH, 'w') as file:
        yaml.dump(core_config_yaml, file)

    robot_config_yaml = {
        'id': id,
        'ip': ip
    }
    with open(HOST_ROBOT_CONFIG_PATH, 'w') as file:
        yaml.dump(robot_config_yaml, file)

    print('>> DONE')



if __name__ == '__main__':
    main()