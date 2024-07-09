#include "arduino_data/arduino_data.h"
#include <iostream>
#include <thread>
#include <future>

int main() {
    rclcpp::init(argc, argv);
    auto arduino_data_node = std::make_shared<rclcpp::Node>("arduino_data_node");

    imu_pub = arduino_data_node->create_publisher<sensor_msgs::msg::Imu>("Arduino_Data/imu", 1);
    button_pub = arduino_data_node->create_publisher<std_msgs::msg::String>("Arduino_Data/button", 1);

    Arduino_Data::initialize();

    while (rclcpp::ok())
    {
        Arduino_Data::deviceLoop();
        if(data_ready)
            Arduino_Data::publishData();
        rclcpp::spinOnce(arduino_data_node);
    }
    close(fd);
    RCLCPP_WARN("Ros shutdown, proceeding to close the port");

    return 0;
}
