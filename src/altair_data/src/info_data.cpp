#include "altair_data/info_data.hpp"



altair::InfoData::InfoData() {
    
    this->core_config   = YAML::LoadFile("/altair-os/src/altair_data/config/core_config.yaml");
    this->robot_config  = YAML::LoadFile("/altair-os/src/altair_data/config/robot_config.yaml");
    this->joint_config  = YAML::LoadFile("/altair-os/src/altair_data/config/joint_config.yaml");

    this->altair_os_path            = this->core_config["os_path"].as<std::string>();
    this->altair_controllers_path   = this->core_config["altair_controllers_path"].as<std::string>();
    this->altair_interfaces_path    = this->core_config["altair_interfaces_path"].as<std::string>();
    this->altair_data_path          = this->core_config["altair_data_path"].as<std::string>();
    this->altair_motion_path        = this->core_config["altair_motion_path"].as<std::string>();
    this->altair_vision_path        = this->core_config["altair_vision_path"].as<std::string>();
    this->altair_main_path          = this->core_config["altair_main_path"].as<std::string>();

    this->id    = this->robot_config["id"].as<std::string>();
    this->ip    = this->robot_config["ip"].as<std::string>();
}