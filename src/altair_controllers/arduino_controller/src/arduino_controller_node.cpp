#include "arduino_controller/arduino_controller.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true);

  auto nh = make_shared<Arduino_Data>();

  nh->init();

  rclcpp::Time current_time = nh->get_clock()->now();
  rclcpp::Duration period = nh->get_clock()->now() - current_time;
  bool first_update = true;
  float control_loop_hz = 500.0;
//   nh->get_parameter("control_loop_hz", control_loop_hz);
  rclcpp::Rate rate(control_loop_hz);
  rclcpp::Time stop_time;
  bool shut_down_started = false;
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(nh);

  while (rclcpp::ok()) {
    nh->read();
    exec.spin_some();
    rate.sleep();
  }

  RCLCPP_WARN(rclcpp::get_logger("arduino_data_node"), "Ros shutdown, proceeding to close the port");

  return 0;
}
