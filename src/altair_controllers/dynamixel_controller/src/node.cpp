#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>

#include "dynamixel_controller/dynamixel_controller.h"


int main(int argc, char** argv) {
  // initialize ros
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("Dynamixel_Controller", options);


  // create hardware interfaces
  altair_hardware::Dynamixel_Controller hw(nh);
  if (!hw.init()) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to initialize hardware interface.");
    return 0;
  }

  // Start control loop
  rclcpp::Time current_time = nh->get_clock()->now();
  rclcpp::Duration period = nh->get_clock()->now() - current_time;
  bool first_update = true;
  float control_loop_hz = 500.0;
  rclcpp::Rate rate(control_loop_hz);
  rclcpp::Time stop_time;
  bool shut_down_started = false;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(nh);

  while (rclcpp::ok()) {
    // read
    hw.read();

    // Write
    hw.write();
    exec.spin_some();
    rate.sleep();
  }
  return 0;

  RCLCPP_WARN(rclcpp::get_logger("arduino_data_node"), "Ros shutdown, proceeding to close the port");
}
