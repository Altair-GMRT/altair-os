#include "altair_interfaces/app_launcher_node.hpp"



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    altair::app_launch();

    while(rclcpp::ok());
    rclcpp::shutdown();

    return 0;
}