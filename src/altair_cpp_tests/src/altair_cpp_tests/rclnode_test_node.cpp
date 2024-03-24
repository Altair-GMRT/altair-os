#include "altair_cpp_tests/rclnode_test_node.hpp"



altair::RclNodeTestNode::RclNodeTestNode() : rclcpp::Node("RclNodeTestNode") {
    this->declare_parameter("pub_msg", rclcpp::PARAMETER_STRING);
    this->declare_parameter("pub_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("sub_topic", rclcpp::PARAMETER_STRING);

    this->pub_msg   = this->get_parameter("pub_msg").as_string();
    this->pub_topic = this->get_parameter("pub_topic").as_string();
    this->sub_topic = this->get_parameter("sub_topic").as_string();

    this->subscriber = this->create_subscription<std_msgs::msg::String>(
        this->sub_topic.c_str(),
        1000,
        std::bind(&altair::RclNodeTestNode::subCallback, this, _1)
    );

    this->publisher = this->create_publisher<std_msgs::msg::String>(
        this->pub_topic.c_str(),
        1000
    );

    this->pub_timer = this->create_wall_timer(
        1s,
        std::bind(&altair::RclNodeTestNode::pubTimerCallback, this)
    );
}



altair::RclNodeTestNode::~RclNodeTestNode() {
    RCLCPP_INFO(this->get_logger(), "RclNodeTestNode destroyed");
}



void altair::RclNodeTestNode::subCallback(const std_msgs::msg::String &msg) {
    RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->sub_topic.c_str(), msg.data.c_str());
}



void altair::RclNodeTestNode::pubTimerCallback() {
    auto msg = std_msgs::msg::String();
    msg.data = this->pub_msg;
    this->publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "message sent to %s", this->pub_topic.c_str());
}