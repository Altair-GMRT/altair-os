#ifndef altair_hardware_INCLUDE_altair_hardware_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_
#define altair_hardware_INCLUDE_altair_hardware_DYNAMIXEL_SERVO_HARDWARE_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <bitset>
#include <string>
#include <map>
#include <fstream>

#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "dynamixel_workbench_toolbox/dynamixel_driver.h"
#include "yaml-cpp/yaml.h"
// #include "robot.h"

namespace altair_hardware {
template <typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() - 1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

enum ControllerMode { MotionModuleMode, DirectControlMode };

struct State {
  State() : position(0), velocity(0), effort(0) {
  }
  double position;
  double velocity;
  double effort;
};

struct Joint {
  std::string name;
  State current;
  State goal;
};

struct Servo {
  std::string name;
  int id;
  double joint_offset;
  std::string model;
};

struct Robot {
  std::string port_name;
  int baudrate;
  double protocol;
  std::string default_joint;
  std::map<int, Servo> joint_map;
};

// altair_hardware::Robot* RobotObject;

class Dynamixel_Controller {
public:
  explicit Dynamixel_Controller(rclcpp::Node::SharedPtr nh);
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<DynamixelWorkbench> driver_;
  std::shared_ptr<Robot> robot_;

  bool init();
  void read();
  void write();
  bool setupDevice();
  bool loadOffsetConfig();
  bool loadDynamixelConfig();
  bool loadRobotConfig();
  bool ping();
  int ping(const std::string joint_name, uint8_t* error = 0);
  int ping(const std::string joint_name, uint16_t* model_number, uint8_t* error = 0);
  std::string robot_file_path;
  void setTorqueCb(std_msgs::msg::Bool::SharedPtr enabled);
  void JointGoalCb(const sensor_msgs::msg::JointState& command_msg);
  // std::shared_ptr<RobotisController> RobotisController_;

  const char* port_name;
  uint8_t baudrate;
  const char* log;

  unsigned int joint_count_;

  std::vector<std::string> joint_names_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> joint_offsets_;
  std::map<std::string, int> joint_map_;

  std::vector<int32_t> goal_torque_individual_;
  std::vector<int32_t> goal_position_;
  std::vector<int32_t> goal_effort_;
  std::vector<int32_t> goal_velocity_;
  std::vector<int32_t> goal_acceleration_;
  std::vector<int32_t> current_position_;
  std::vector<int32_t> current_velocity_;
  std::vector<int32_t> current_effort_;
  std::vector<int32_t> current_pwm_;
  std::vector<int32_t> current_input_voltage_;
  std::vector<int32_t> current_temperature_;
  std::vector<uint8_t> current_error_;
  int32_t* data_sync_read_positions_;
  int32_t* data_sync_read_temperature_;
  int32_t* data_sync_read_velocities_;
  int32_t* data_sync_read_efforts_;
  int32_t* data_sync_read_pwms_;
  int32_t* data_sync_read_error_;
  int32_t* sync_write_goal_position_;
  int32_t* sync_write_goal_velocity_;
  int32_t* sync_write_profile_velocity_;
  int32_t* sync_write_profile_acceleration_;
  std::vector<uint8_t> sync_read_all_data_;

  bool torqueless_mode_;

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_torque_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_command_;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pwm_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr temperature_pub_;

  sensor_msgs::msg::JointState joint_state_msg_;
  std_msgs::msg::Int32MultiArray temperature_msg_;
  sensor_msgs::msg::JointState pwm_msg_;

  bool DEBUG_PRINT;
};
/* 
class RobotisController {
public:
  int read(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data, uint8_t* error = 0);
  int readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t* data, uint8_t* error = 0);

  int read1Byte(const std::string joint_name, uint16_t address, uint8_t* data, uint8_t* error = 0);
  int read2Byte(const std::string joint_name, uint16_t address, uint16_t* data, uint8_t* error = 0);
  int read4Byte(const std::string joint_name, uint16_t address, uint32_t* data, uint8_t* error = 0);

  int write(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data, uint8_t* error = 0);
  int writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data, uint8_t* error = 0);

  int write1Byte(const std::string joint_name, uint16_t address, uint8_t data, uint8_t* error = 0);
  int write2Byte(const std::string joint_name, uint16_t address, uint16_t data, uint8_t* error = 0);
  int write4Byte(const std::string joint_name, uint16_t address, uint32_t data, uint8_t* error = 0);

  int regWrite(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data, uint8_t* error = 0);
};
*/
}  // namespace altair_hardware

#endif
