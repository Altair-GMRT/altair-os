#include "arduino_data/arduino_data.h"
#include <iostream>
#include <thread>



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto arduino_data_node = make_shared<Arduino_Data>();
    // arduino_data_node->declare_parameter("portName");
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(arduino_data_node);
    }
    
    
    RCLCPP_WARN(rclcpp::get_logger("arduino_data_node"), "Ros shutdown, proceeding to close the port");

    return 0;
}
