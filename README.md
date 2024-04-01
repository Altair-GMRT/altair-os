# **altair-os**
A ROS2-based framework for Altair Team of GMRT robot development.



## **1. Installation**
We use Docker to develop the robot since Raspberry Pi 5 does not support Ubuntu 22.04, resulting in no ROS2 distro available for our current operating system. Note that we can directly ```colcon build``` this repository if we are working on a machine that matches the requirements for ROS2 Humble. Or else, do the following steps to begin working with this repository:

### **1.1 Initialization**
This process is used for 3 things: creating a unique ID or namespace for our machine (robot), defining the frequency of the master clock (used for many callback periods), and getting the local network IP, if one exists. This helps a lot when we work on multiple robots on the same local network. In the container's bash terminal, run the following:

```console
# On host machine
python3 altair_setup.py
```

We will be asked for an ID/namespace and a frequency to be given to the machine.

### **1.2 Build the Images** 
Run the following commands to build the Docker image required:

```console
# On host machine
sudo docker build -t altair-os .
``` 

### **1.3 Start the Containers**
Run the following command:

```console
# On host machine
sudo docker compose up -d
```



## **2. Basic Usage**

### **2.1 Developing and Building**
Please note that the container ```altair-os``` volume-bind the ```src/``` and ```web/``` directory. Therefore, we might develop the source codes on our local or host machine and build the project on the Docker container. Use the following command to access the Docker container interactively on our terminal:

```console
# On host machine
sudo docker exec -it altair-os bash
```

#### **a. For Building the ROS Project**
In the container's bash terminal, run the following:

```console
# On Docker container
colcon build
```

Of course, we can do the other things inside the container's bash terminal if needed. 

#### **b. For Building the Web App**
It is more convenient to develop and build the web app on our own machine. But since the ```rclnodejs``` requires ROS2 environment to be sourced, make sure we have ROS2 installed in our machine. Run the following command everytime the web app project is reinstalled:

```console
# On host machine
colcon build --packages-select altair_interfaces
source install/setup.bash
cd web/
npm install
npx generate-ros-messages
```

> [!IMPORTANT]
> Use the same node version (20.11.1).

### **2.2 Launches**
Remember to source the workspace's bash file everytime we open a new terminal with:

```console
# On Docker container
source install/setup.bash
```

All launch files are contained inside ```altair_main``` package. Go to ```src/altair_main/launch/``` to see all the available launch files. Proceed with the following comment to run the launch file:

```console
# On Docker container
ros2 launch altair_main <launch_file_name.py>
```