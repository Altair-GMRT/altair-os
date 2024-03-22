#include "altair_cpp_tests/custom_msg_test_node.hpp"



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altair::CustomMsgTestNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}