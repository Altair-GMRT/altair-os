import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs 



class RclNodeTestNode(Node):

    def __init__(self) -> None:
        super().__init__('RclNodeTestNode')

        self.declare_parameter('pub_msg', rclpy.Parameter.Type.STRING)
        self.declare_parameter('pub_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('sub_topic', rclpy.Parameter.Type.STRING)

        self.pub_msg    = self.get_parameter('pub_msg').value
        self.pub_topic  = self.get_parameter('pub_topic').value
        self.sub_topic  = self.get_parameter('sub_topic').value

        self.subscriber = self.create_subscription(
            msg_type    = std_msgs.String,
            topic       = self.sub_topic,
            callback    = self.subCallback,
            qos_profile = 1000
        )

        self.publisher = self.create_publisher(
            msg_type    = std_msgs.String,
            topic       = self.pub_topic,
            qos_profile = 1000
        )

        self.pub_timer = self.create_timer(1., self.pubTimerCallback)



    def subCallback(self, msg:std_msgs.String) -> None:
        self.get_logger().info(f'[{self.sub_topic}]: {msg.data}')



    def pubTimerCallback(self) -> None:
        msg         = std_msgs.String()
        msg.data    = self.pub_msg
        self.publisher.publish(msg)
        
        self.get_logger().info(f'message sent to {self.pub_topic}')