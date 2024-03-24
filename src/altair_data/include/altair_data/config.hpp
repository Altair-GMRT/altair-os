#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <yaml-cpp/yaml.h>

namespace altair {

    struct Config {

        YAML::Node  core_config,
                    robot_config,
                    joint_config;

        std::string id,
                    ip;

        Config();
    };
}

#endif