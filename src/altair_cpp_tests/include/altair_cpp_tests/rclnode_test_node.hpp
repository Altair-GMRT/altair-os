#ifndef __RCLNODE_TEST_NODE_HPP__
#define __RCLNODE_TEST_NODE_HPP__

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace altair {

    class RclNodeTestNode : public rclcpp::Node {
        
        public:
            RclNodeTestNode();
            ~RclNodeTestNode();

        private:
            std::string pub_msg,
                        pub_topic,
                        sub_topic;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  subscriber;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     publisher;
            rclcpp::TimerBase::SharedPtr                            pub_timer;

            void subCallback(const std_msgs::msg::String &msg);
            void pubTimerCallback();
    };
}

#endif