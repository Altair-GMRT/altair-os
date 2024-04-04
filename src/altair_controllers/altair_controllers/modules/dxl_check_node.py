import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *



class DXLCheckNode(Node):

    def __init__(self) -> None:
        super().__init__('DXLCheckNode')
        self.get_logger()