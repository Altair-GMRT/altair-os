import yaml
import rclpy
from rclpy.node import Node
from .dxl_type.dxl_controller import *



class AppControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('AppControllerNode')

        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('u2d2_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('baudrate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('joint_config_path', rclpy.Parameter.Type.STRING)

        self.ID             = self.get_parameter('id').value
        self.U2D2_PORT      = self.get_parameter('u2d2_port').value
        self.BAUDRATE       = self.get_parameter('baudrate').value
        self.JOINT_CONFIG   = yaml.safe_load(open(self.get_parameter('joint_config_path').value, 'r'))
        
        self.mx28_id    = []
        self.mx28_name  = dict(())
        self.mx64_id    = []
        self.mx64_name  = dict(())

        self.porthandler    = dxl.PortHandler(self.U2D2_PORT)
        self.packethandler  = dxl.PacketHandler(2.0)

        try:
            self.porthandler.openPort()
            self.get_logger().info(f'Port {self.U2D2_PORT} opened successfully.')

        except:
            self.get_logger().error(f'Failed to open port on {self.U2D2_PORT}')
            self.get_logger().error(f'Try "sudo chmod a+rw {self.U2D2_PORT}"')
            quit()

        try:
            self.porthandler.setBaudRate(self.BAUDRATE)
            self.get_logger().info(f'Baudrate set to {self.BAUDRATE}')

        except:
            self.get_logger().error(f'Failed to set baudrate to {self.BAUDRATE}')
            quit()

        self.dxl_controller = DXLController(
            self.porthandler, 
            self.packethandler
        )



    def dxlSearch(self) -> None:
        for i in range(self.JOINT_CONFIG['dxl_num']):
            dxl_id      = self.JOINT_CONFIG['dxl_id'][i]
            dxl_type    = self.JOINT_CONFIG['dxl_type'][i]
            joint_name  = self.JOINT_CONFIG['joint_name'][i]

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