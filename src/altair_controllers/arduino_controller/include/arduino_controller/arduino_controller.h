#ifndef Controller_H
#define Controller_H



#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/char.hpp"
#include <string>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>

#include "robotis_math/robotis_math_base.h"
using namespace robotis_framework;
using namespace std;

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <fstream>
#include <sys/ioctl.h>
#include <linux/serial.h>

class Arduino_Data : public rclcpp::Node
{

public:
  Arduino_Data() : Node("arduino_data_node") 
  {
    // get_parameter_or<std::string>("portName", portName, "/dev/ttyUSB0 "); 
    
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("arduino_data/imu", 1);
    button_pub = this->create_publisher<std_msgs::msg::Char>("arduino_data/button", 1);


  }
  ~Arduino_Data();
  bool init();
  void read();

private:
  sensor_msgs::msg::Imu imu_msg;
  std_msgs::msg::Char button_msg;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr button_pub;

  int fd;
  const char *portName;
  int baudRate;
  int waitTimeout;
  int timeOut_ctr = 0;
  bool broadcast = broadcast;
  bool reconnect = false;
  bool dev_connected = false;

  std::ifstream serial(const char* portName);

  double offset;

  int init_argc_;
  char** init_argv_;
  int time;
  

  bool data_ready = false;
  float roll, pitch, yaw, gyroRoll, gyroPitch, gyroYaw, accelX, accelY, accelZ, temp_data;
  string read_buffer;
  float button;
  char type;
  float value;


  void configure_port(int fd);
  bool initialize();
  void process();
  
};

#endif // Controller_H
