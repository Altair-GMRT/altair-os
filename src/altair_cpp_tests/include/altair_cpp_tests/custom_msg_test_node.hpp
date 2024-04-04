#ifndef __CUSTOM_MSG_TEST_NODE_HPP__
#define __CUSTOM_MSG_TEST_NODE_HPP__

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "altair_interfaces/msg/test_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace altair {

    class CustomMsgTestNode : public rclcpp::Node {
        
        public:
            CustomMsgTestNode();
            ~CustomMsgTestNode();

        private:
            std::string pub_topic,
                        sub_topic,
                        name;
            int         age;
            double      score;

            rclcpp::Subscription<altair_interfaces::msg::TestMessage>::SharedPtr    subscriber;
            rclcpp::Publisher<altair_interfaces::msg::TestMessage>::SharedPtr       publisher;
            rclcpp::TimerBase::SharedPtr                                            pub_timer;

            void subCallback(const altair_interfaces::msg::TestMessage &msg);
            void pubTimerCallback();
    };
}

#endif