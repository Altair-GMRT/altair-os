#include "altair_cpp_tests/custom_msg_test_node.hpp"



altair::CustomMsgTestNode::CustomMsgTestNode() : rclcpp::Node("CustomMsgTestNode"){
    this->declare_parameter("name", rclcpp::PARAMETER_STRING);
    this->declare_parameter("age", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("score", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("pub_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("sub_topic", rclcpp::PARAMETER_STRING);

    this->name      = this->get_parameter("name").as_string();
    this->age       = this->get_parameter("age").as_int();
    this->score     = this->get_parameter("score").as_double();
    this->pub_topic = this->get_parameter("pub_topic").as_string();
    this->sub_topic = this->get_parameter("sub_topic").as_string();

    this->subscriber = this->create_subscription<altair_interfaces::msg::TestMessage>(
        this->sub_topic.c_str(),
        1000,
        std::bind(&altair::CustomMsgTestNode::subCallback, this, _1)
    );

    this->publisher = this->create_publisher<altair_interfaces::msg::TestMessage>(
        this->pub_topic.c_str(),
        1000
    );

    this->pub_timer = this->create_wall_timer(
        1s,
        std::bind(&altair::CustomMsgTestNode::pubTimerCallback, this)
    );
}



altair::CustomMsgTestNode::~CustomMsgTestNode() {
    RCLCPP_INFO(this->get_logger(), "CustomMsgTestNode destroyed");
}



void altair::CustomMsgTestNode::subCallback(const altair_interfaces::msg::TestMessage &msg) {
    RCLCPP_INFO(this->get_logger(), "[%s]: name = %s, age = %d, score = %lf", this->sub_topic.c_str(), msg.name.c_str(), msg.age, msg.score);
}



void altair::CustomMsgTestNode::pubTimerCallback() {
    auto msg = altair_interfaces::msg::TestMessage();
    msg.name    = this->name;
    msg.age     = this->age;
    msg.score   = this->score;
    this->publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "message sent to %s", this->pub_topic.c_str());
}