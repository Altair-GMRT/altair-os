#ifndef Controller_H
#define Controller_H



#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.h"
#include "std_msgs/msg/string.h"  
#include <string>
#include <string.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/convert.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include "robotis_math/robotis_linear_algebra.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstdlib>
#include <mutex>
#include <fstream>

class Arduino_Data:
{

public:
  explicit Arduino_Data(int argc, char** argv);
  ~Arduino_Data();

  enum rx_arduino_state {
    Ardu_Start_U,
    Ardu_Start_G,
    Ardu_Start_M,
    Ardu_Start_D,
    Ardu_Start_I,
    Ardu_Start_Y,
    Ardu_Start_P,
    Ardu_Start_C,
    Ardu_Start_T,
    Ardu_Start_B,
  };


signals:
    void finished();
    void start();

public slots:
  void run();

private:
  int fd;
  auto devicedata;
  std::mutex mutex;
  bool currentPortNameChanged;
  std::string portName, currentPortName;
  int baudRate;
  int waitTimeout;
  int timeOut_ctr;
  bool broadcast;
  bool reconnect;
  bool dev_connected;
  std::regex regexPattern("[UGMDIYPCTBA]");

  double offset;

  int init_argc_;
  char** init_argv_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub;

  bool data_ready;
  rx_arduino_state arduino_state;
  double roll,pitch,yaw,gyroRoll,gyroPitch,gyroYaw,accelX,accelY,accelZ, temp_data;
  std::string read_buffer;
  char type;
  int value;
  int button, prev_button;
  bool readyToSend;
  std::map<int, std::string> button_mapping;

  boost::asio::io_service io;

  void debugDevice();
  void publishData();
  bool initialize();
  void deviceCheck();
  void deviceLoop();
  void loadButton(const std::string path);


};

#endif // Controller_H
