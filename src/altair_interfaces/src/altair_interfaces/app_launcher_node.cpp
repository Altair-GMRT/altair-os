#include "altair_interfaces/app_launcher_node.hpp"



void altair::app_launch() {
    altair::Config config = altair::Config();

    RCLCPP_INFO(rclcpp::get_logger("app_launcher"), "Starting Altair App on %s:3000/%s", config.ip.c_str(), config.id.c_str());
    system("node /altair-os/web/app.js");
}