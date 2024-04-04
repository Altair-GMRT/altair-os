from .modules.app_controller_node import *



def main(args=None) -> None:
    rclpy.init()
    node = AppControllerNode()
    node.dxlSearch()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()