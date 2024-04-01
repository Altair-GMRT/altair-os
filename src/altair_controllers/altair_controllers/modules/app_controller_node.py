import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *
import altair_interfaces.msg as altair_interfaces



class AppControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('AppControllerNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_baudrate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_u2d2_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('dxl_num', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('dxl_id', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('dxl_type', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joint_name', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('master_clock', rclpy.Parameter.Type.DOUBLE)

        self.ID             = self.get_parameter('id').value
        self.DXL_BAUDRATE   = self.get_parameter('dxl_baudrate').value
        self.DXL_U2D2_PORT  = self.get_parameter('dxl_u2d2_port').value
        self.DXL_NUM        = self.get_parameter('dxl_num').value
        self.DXL_ID         = self.get_parameter('dxl_id').value
        self.DXL_TYPE       = self.get_parameter('dxl_type').value
        self.JOINT_NAME     = self.get_parameter('joint_name').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        
        self.mx28_id    = []
        self.mx28_name  = dict(())
        self.mx64_id    = []
        self.mx64_name  = dict(())

        self.porthandler    = dxl.PortHandler(self.DXL_U2D2_PORT)
        self.packethandler  = dxl.PacketHandler(2.0)

        try:
            self.porthandler.openPort()
            self.get_logger().info(f'Port {self.DXL_U2D2_PORT} opened successfully.')

        except:
            self.get_logger().error(f'Failed to open port on {self.DXL_U2D2_PORT}')
            self.get_logger().error(f'Try "sudo chmod a+rw {self.DXL_U2D2_PORT}"')
            quit()

        try:
            self.porthandler.setBaudRate(self.DXL_BAUDRATE)
            self.get_logger().info(f'Baudrate set to {self.DXL_BAUDRATE}')

        except:
            self.get_logger().error(f'Failed to set baudrate to {self.DXL_BAUDRATE}')
            quit()

        self.dxl_controller = DXLController(
            self.porthandler, 
            self.packethandler
        )

        self.goal_position_sub = self.create_subscription(
            msg_type    = altair_interfaces.JointPosition,
            topic       = f'{self.ID}/goal_position',
            callback    = self.goalPositionSubCallback,
            qos_profile = 1000
        )

        self.goal_velocity_sub = self.create_subscription(
            msg_type    = altair_interfaces.JointVelocity,
            topic       = f'{self.ID}/goal_velocity',
            callback    = self.goalVelocitySubCallback,
            qos_profile = 1000
        )

        self.goal_torque_sub = self.create_subscription(
            msg_type    = altair_interfaces.JointTorque,
            topic       = f'{self.ID}/goal_torque',
            callback    = self.goalTorqueSubCallback,
            qos_profile = 1000
        )

        self.joint_sensor_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointSensor,
            topic       = f'{self.ID}/joint_sensor',
            qos_profile = 10
        )

        self.present_position_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointPosition,
            topic       = f'{self.ID}/present_position',
            qos_profile = 10
        )

        self.present_velocity_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointVelocity,
            topic       = f'{self.ID}/present_velocity',
            qos_profile = 10
        )

        self.present_torque_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointTorque,
            topic       = f'{self.ID}/present_torque',
            qos_profile = 10
        )

        self.pub_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.pubTimerCallback
        )



    def dxlSearch(self) -> None:
        for i in range(self.DXL_NUM):
            dxl_id      = self.DXL_ID[i]
            dxl_type    = self.DXL_TYPE[i]
            joint_name  = self.JOINT_NAME[i]

            if dxl_type == 'MX28':
                self.mx28_id.append(dxl_id)
                self.mx28_name.update({dxl_id: joint_name})

            elif dxl_type == 'MX64':
                self.mx64_id.append(dxl_id)
                self.mx64_name.update({dxl_id: joint_name})

        for id in self.mx28_id:
            res = self.dxl_controller.read(
                address = DXL_MX28_MODEL_NUMBER_ADDR,
                size    = DXL_MX28_MODEL_NUMBER_SIZE,
                dxl_id  = id
            )

            if res >= DXL_OK:
                self.get_logger().info(f'[ID:{id} ({self.mx28_name[id]})]: Connection established.')

            else:
                self.get_logger().error(f'[ID:{id} ({self.mx28_name[id]})]: Failed to connect.')

        for id in self.mx64_id:
            res = self.dxl_controller.read(
                address = DXL_MX64_MODEL_NUMBER_ADDR,
                size    = DXL_MX64_MODEL_NUMBER_SIZE,
                dxl_id  = id
            )

            if res >= DXL_OK:
                self.get_logger().info(f'[ID:{id} ({self.mx64_name[id]})]: Connection established.')

            else:
                self.get_logger().error(f'[ID:{id} ({self.mx64_name[id]})]: Failed to connect.')



    def readTorque(self) -> None:
        pass



    def readPresentPosition(self) -> None:
        pass



    def readPresentVelocity(self) -> None:
        pass



    def readPresentLoad(self) -> None:
        pass



    def readPresentInputVoltage(self) -> None:
        pass



    def readPresentTemperature(self) -> None:
        pass



    def writeTorque(self) -> None:
        pass



    def writeGoalPosition(self) -> None:
        pass



    def writeGoalVelocity(self) -> None:
        pass



    def goalTorqueSubCallback(self, msg:altair_interfaces.JointTorque) -> None:
        pass



    def goalPositionSubCallback(self, msg:altair_interfaces.JointPosition) -> None:
        pass



    def goalVelocitySubCallback(self, msg:altair_interfaces.JointVelocity) -> None:
        pass



    def pubTimerCallback(self) -> None:
        pass