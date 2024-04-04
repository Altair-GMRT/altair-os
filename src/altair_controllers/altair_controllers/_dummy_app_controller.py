from .modules._dummy_app_controller_node import *



def main(args=None) -> None:
    rclpy.init()
    node = DummyAppControllerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()