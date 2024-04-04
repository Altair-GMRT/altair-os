import rclpy
from .modules.custom_msg_test_node import CustomMsgTestNode



def main(args=None) -> None:
    rclpy.init()
    node = CustomMsgTestNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()