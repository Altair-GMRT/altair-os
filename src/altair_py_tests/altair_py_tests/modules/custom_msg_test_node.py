import rclpy
from rclpy.node import Node
import altair_interfaces.msg as altair_interfaces



class CustomMsgTestNode(Node):

    def __init__(self) -> None:
        super().__init__('CustomMsgTestNode')

        self.declare_parameter('name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('age', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('score', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pub_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('sub_topic', rclpy.Parameter.Type.STRING)

        self.name       = self.get_parameter('name').value
        self.age        = self.get_parameter('age').value
        self.score      = self.get_parameter('score').value
        self.pub_topic  = self.get_parameter('pub_topic').value
        self.sub_topic  = self.get_parameter('sub_topic').value

        self.subscriber = self.create_subscription(
            msg_type    = altair_interfaces.TestMessage,
            topic       = self.sub_topic,
            callback    = self.subCallback,
            qos_profile = 1000
        )

        self.publisher = self.create_publisher(
            msg_type    = altair_interfaces.TestMessage,
            topic       = self.pub_topic,
            qos_profile = 1000
        )

        self.pub_timer = self.create_timer(1., self.pubTimerCallback)



    def subCallback(self, msg:altair_interfaces.TestMessage) -> None:
        self.get_logger().info(f'[{self.sub_topic}]: name = {msg.name}, age = {msg.age}, score = {msg.score}')



    def pubTimerCallback(self) -> None:
        msg         = altair_interfaces.TestMessage()
        msg.name    = self.name
        msg.age     = self.age
        msg.score   = self.score
        self.publisher.publish(msg)
        
        self.get_logger().info(f'message sent to {self.pub_topic}')