import rclpy
from .modules.rclnode_test_node import RclNodeTestNode



def main(args=None) -> None:
    rclpy.init()
    node = RclNodeTestNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()