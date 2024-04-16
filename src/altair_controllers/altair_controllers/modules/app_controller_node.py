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
        
        self.mx28_id_list   = []
        self.mx28_id_set    = set(())
        self.mx28_name      = dict(())
        self.mx64_id_list   = []
        self.mx64_id_set    = set(())
        self.mx64_name      = dict(())

        self.write_torque   = False
        self.write_position = False
        self.write_velocity = False

        self.goal_torque    = []
        self.goal_position  = []
        self.goal_velocity  = []

        self.present_torque         = []
        self.present_position       = []
        self.present_velocity       = []
        self.present_load           = []
        self.present_voltage        = []
        self.present_temperature    = []

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

        self.goal_torque_sub = self.create_subscription(
            msg_type    = altair_interfaces.JointTorque,
            topic       = f'{self.ID}/goal_torque',
            callback    = self.goalTorqueSubCallback,
            qos_profile = 1000
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

        self.present_torque_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointTorque,
            topic       = f'{self.ID}/present_torque',
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

        self.joint_sensor_pub = self.create_publisher(
            msg_type    = altair_interfaces.JointSensor,
            topic       = f'{self.ID}/joint_sensor',
            qos_profile = 10
        )

        self.controller_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.controllerTimerCallback
        )



    def dxlSearch(self) -> None:
        for i in range(self.DXL_NUM):
            dxl_id      = self.DXL_ID[i]
            dxl_type    = self.DXL_TYPE[i]
            joint_name  = self.JOINT_NAME[i]

            if dxl_type == 'MX28':
                self.mx28_id_list.append(dxl_id)
                self.mx28_id_set.add(dxl_id)
                self.mx28_name.update({dxl_id: joint_name})

            elif dxl_type == 'MX64':
                self.mx64_id_list.append(dxl_id)
                self.mx64_id_set.add(dxl_id)
                self.mx64_name.update({dxl_id: joint_name})

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res = self.dxl_controller.read(
                    address = DXL_MX28_MODEL_NUMBER_ADDR,
                    size    = DXL_MX28_MODEL_NUMBER_SIZE,
                    dxl_id  = self.DXL_ID[i]
                )

                if res >= DXL_OK:
                    self.get_logger().info(f'[ID:{self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Connection established.')

                else:
                    self.get_logger().error(f'[ID:{self.DXL_ID[i]} ({self.mx28_name[self.DXL_ID[i]]})]: Failed to connect.')
                    self.get_logger().info('Make sure all the servo are connected properly. Terminating.')
                    
                    self.destroy_node()
                    rclpy.shutdown()
                    quit()

            elif self.DXL_ID[i] in self.mx64_id_set:
                res = self.dxl_controller.read(
                    address = DXL_MX64_MODEL_NUMBER_ADDR,
                    size    = DXL_MX64_MODEL_NUMBER_SIZE,
                    dxl_id  = self.DXL_ID[i]
                )

                if res >= DXL_OK:
                    self.get_logger().info(f'[ID:{self.DXL_ID[i]} ({self.mx64_name[self.DXL_ID[i]]})]: Connection established.')

                else:
                    self.get_logger().error(f'[ID:{self.DXL_ID[i]} ({self.mx64_name[self.DXL_ID[i]]})]: Failed to connect.')
                    self.get_logger().info('Make sure all the servo are connected properly. Terminating.')
                    
                    self.destroy_node()
                    rclpy.shutdown()
                    quit()
            


    def readTorque(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            error_bypass        = True,
            error_handle_val    = 2
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_TORQUE_ENABLE_ADDR,
            size    = DXL_MX64_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx64_id_list,
            error_bypass        = True,
            error_handle_val    = 2
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def readPresentPosition(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_PRESENT_POSITION_ADDR,
            size    = DXL_MX28_PRESENT_POSITION_SIZE,
            dxl_id  = self.mx28_id_list,
            error_bypass        = True,
            error_handle_val    = 999999999
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_PRESENT_POSITION_ADDR,
            size    = DXL_MX64_PRESENT_POSITION_SIZE,
            dxl_id  = self.mx64_id_list,
            error_bypass        = True,
            error_handle_val    = 999999999
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def readPresentVelocity(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_PRESENT_VELOCITY_ADDR,
            size    = DXL_MX28_PRESENT_VELOCITY_SIZE,
            dxl_id  = self.mx28_id_list,
            error_bypass        = True,
            error_handle_val    = 999999999
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_PRESENT_VELOCITY_ADDR,
            size    = DXL_MX64_PRESENT_VELOCITY_SIZE,
            dxl_id  = self.mx64_id_list,
            error_bypass        = True,
            error_handle_val    = 999999999
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def readPresentLoad(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_PRESENT_LOAD_ADDR,
            size    = DXL_MX28_PRESENT_LOAD_SIZE,
            dxl_id  = self.mx28_id_list
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_PRESENT_LOAD_ADDR,
            size    = DXL_MX64_PRESENT_LOAD_SIZE,
            dxl_id  = self.mx64_id_list
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def readPresentInputVoltage(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_PRESENT_INPUT_VOLTAGE_ADDR,
            size    = DXL_MX28_PRESENT_INPUT_VOLTAGE_SIZE,
            dxl_id  = self.mx28_id_list
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_PRESENT_INPUT_VOLTAGE_ADDR,
            size    = DXL_MX64_PRESENT_INPUT_VOLTAGE_SIZE,
            dxl_id  = self.mx64_id_list
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def readPresentTemperature(self) -> list:
        mx28_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX28_PRESENT_TEMPERATURE_ADDR,
            size    = DXL_MX28_PRESENT_TEMPERATURE_SIZE,
            dxl_id  = self.mx28_id_list
        )

        mx64_res = self.dxl_controller.groupSyncRead(
            address = DXL_MX64_PRESENT_TEMPERATURE_ADDR,
            size    = DXL_MX64_PRESENT_TEMPERATURE_SIZE,
            dxl_id  = self.mx64_id_list
        )

        res         = []
        mx28_iter   = iter(mx28_res)
        mx64_iter   = iter(mx64_res)

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                res.append(next(mx28_iter))

            elif self.DXL_ID[i] in self.mx64_id_set:
                res.append(next(mx64_iter))

        return res



    def writeTorque(self) -> None:
        mx28_param = []
        mx64_param = []

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                mx28_param.append(
                    self.dxl_controller.convert1ByteToDxl(
                        self.goal_torque[i]
                    )
                )

            elif self.DXL_ID[i] in self.mx64_id_set:
                mx64_param.append(
                    self.dxl_controller.convert1ByteToDxl(
                        self.goal_torque[i]
                    )
                )

        mx28_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX28_TORQUE_ENABLE_ADDR,
            size    = DXL_MX28_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = mx28_param
        )

        mx64_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX64_TORQUE_ENABLE_ADDR,
            size    = DXL_MX64_TORQUE_ENABLE_SIZE,
            dxl_id  = self.mx64_id_list,
            params  = mx64_param
        )

        if mx28_res < DXL_OK:
            self.get_logger().error('[MX28 SyncWrite Torque] Failed')

        if mx64_res < DXL_OK:
            self.get_logger().error('[MX64 SyncWrite Torque] Failed')



    def writeGoalPosition(self) -> None:
        mx28_param = []
        mx64_param = []

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                mx28_param.append(
                    self.dxl_controller.convert4ByteToDxl(
                        self.goal_position[i]
                    )
                )

            elif self.DXL_ID[i] in self.mx64_id_set:
                mx64_param.append(
                    self.dxl_controller.convert4ByteToDxl(
                        self.goal_position[i]
                    )
                )

        mx28_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX28_GOAL_POSITION_ADDR,
            size    = DXL_MX28_GOAL_POSITION_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = mx28_param
        )

        mx64_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX64_GOAL_POSITION_ADDR,
            size    = DXL_MX64_GOAL_POSITION_SIZE,
            dxl_id  = self.mx64_id_list,
            params  = mx64_param
        )

        if mx28_res < DXL_OK:
            self.get_logger().error('[MX28 SyncWrite Position] Failed')

        if mx64_res < DXL_OK:
            self.get_logger().error('[MX64 SyncWrite Position] Failed')



    def writeGoalVelocity(self) -> None:
        mx28_param = []
        mx64_param = []

        for i in range(self.DXL_NUM):
            if self.DXL_ID[i] in self.mx28_id_set:
                mx28_param.append(
                    self.dxl_controller.convert4ByteToDxl(
                        self.goal_velocity[i]
                    )
                )

            elif self.DXL_ID[i] in self.mx64_id_set:
                mx64_param.append(
                    self.dxl_controller.convert4ByteToDxl(
                        self.goal_velocity[i]
                    )
                )

        mx28_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX28_PROFILE_VELOCITY_ADDR,
            size    = DXL_MX28_PROFILE_VELOCITY_SIZE,
            dxl_id  = self.mx28_id_list,
            params  = mx28_param
        )

        mx64_res = self.dxl_controller.groupSyncWrite(
            address = DXL_MX64_PROFILE_VELOCITY_ADDR,
            size    = DXL_MX64_PROFILE_VELOCITY_SIZE,
            dxl_id  = self.mx64_id_list,
            params  = mx64_param
        )

        if mx28_res < DXL_OK:
            self.get_logger().error('[MX28 SyncWrite Velocity] Failed')

        if mx64_res < DXL_OK:
            self.get_logger().error('[MX64 SyncWrite Velocity] Failed')



    def goalTorqueSubCallback(self, msg:altair_interfaces.JointTorque) -> None:
        self.write_torque   = True
        self.goal_torque    = msg.val 



    def goalPositionSubCallback(self, msg:altair_interfaces.JointPosition) -> None:
        self.write_position = True
        self.goal_position  = msg.val



    def goalVelocitySubCallback(self, msg:altair_interfaces.JointVelocity) -> None:
        self.write_velocity = True
        self.goal_velocity  = msg.val



    def controllerTimerCallback(self) -> None:
        present_torque_msg      = altair_interfaces.JointTorque()
        present_position_msg    = altair_interfaces.JointPosition()
        present_velocity_msg    = altair_interfaces.JointVelocity()
        joint_sensor_msg        = altair_interfaces.JointSensor()

        present_torque_msg.val          = self.readTorque()
        present_position_msg.val        = self.readPresentPosition()
        present_velocity_msg.val        = self.readPresentVelocity()
        joint_sensor_msg.load           = self.readPresentLoad()
        joint_sensor_msg.voltage        = self.readPresentInputVoltage()
        joint_sensor_msg.temperature    = self.readPresentTemperature()

        self.present_torque_pub.publish(present_torque_msg)
        self.present_position_pub.publish(present_position_msg)
        self.present_velocity_pub.publish(present_velocity_msg)
        self.joint_sensor_pub.publish(joint_sensor_msg)

        if self.write_torque:
            self.write_torque = False
            self.writeTorque()

        if self.write_position:
            self.write_position = False
            self.writeGoalPosition()

        if self.write_velocity:
            self.write_velocity = False
            self.writeGoalVelocity()