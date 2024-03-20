# altair-os
A ROS2-based framework for Altair Team of GMRT robot development.

## 1. Installation and Basic Usage
We use Docker to develop the robot since Raspberry Pi 5 does not support Ubuntu 22.04, resulting in no ROS2 distro available for our current operating system. Note that you can directly ```colcon build``` this repository if you are working on a machine that matches the requirements for ROS2 Humble. Or else, do the following steps to begin working with this repository:

### **1.1 Build the Images** 
Run the following commands to build the Docker image required:

```
sudo docker build -t altair-os . -f Dockerfile
``` 

### **1.2 Build the Containers**
Run the following command:

```
sudo docker compose up -d
```


### **1.3 Developing and Building**
Please note that the container ```altair-os``` volume-bind the ```src/``` and ```web/``` directory. Therefore, you might develop the source codes on your local or host machine and build the project on the Docker container. Use the following command to access the Docker container interactively on your terminal:

```
sudo docker exec -it altair-os bash
```

In the container's bash terminal, run the following:

```
cd altair-os/
colcon build
```

Of course, we can do the other things inside the container's bash terminal if needed. 