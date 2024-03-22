#include "altair_cpp_tests/rclnode_test_node.hpp"



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altair::RclNodeTestNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}