#ifndef __INFO_DATA_HPP__
#define __INFO_DATA_HPP__

#include <string>
#include <yaml-cpp/yaml.h>

namespace altair {

    struct InfoData {

        YAML::Node  core_config,
                    robot_config,
                    joint_config;

        std::string altair_os_path,
                    altair_data_path,
                    altair_interfaces_path,
                    altair_controllers_path,
                    altair_motion_path,
                    altair_vision_path,
                    altair_main_path,
                    id,
                    ip;

        InfoData();
    };
}

#endif