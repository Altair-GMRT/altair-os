Altair Simulation
![Screenshot from 2024-08-01 12-36-43](https://github.com/user-attachments/assets/060fcf49-50ad-46cb-b626-55486e6ed975) no texture \
![Screenshot from 2024-08-05 11-01-20](https://github.com/user-attachments/assets/0a0f50b6-e647-4638-85e0-517dd3e19e8f) with texture \


Still a barebone, as there are no walking program

Note: 

Using available assests provided by robotis op3 and OpenRobotics \
Virtual IMU is working \
Virtual Camera camera is working \
Both sensor can be accessed from topic:
/gazebo/imu \
/gazebo/image_raw

Run:
ros2 launch altair_simulation altair.gazebo.launch.py
