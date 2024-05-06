import rclpy
import altair_interfaces.msg as altair_interfaces
import controller as wbt
import numpy as np
from typing import List



class WebotsOP3Driver:

    def init(self, webots_node, properties) -> None:
        rclpy.init()

        self.WEBOTS_BASIC_TIMESTEP  = int(properties['webots_basic_timestep'])
        self.ROBOT_NAME             = str(properties['robot_name'])
        self.JOINT_NUM              = int(properties['joint_num'])
        self.JOINTS                 = str(properties['joints']).split()
        self.POS_SENSORS            = str(properties['position_sensors']).split()

        self.node                                   = rclpy.create_node(self.ROBOT_NAME + '_driver')
        self.robot:wbt.Robot                        = webots_node.robot
        self.joint:List[wbt.Motor]                  = []
        self.pos_sensor:List[wbt.PositionSensor]    = []
        self.accel:wbt.Accelerometer                = None
        self.gyro:wbt.Gyro                          = None

        for i in range(self.JOINT_NUM):
            self.joint.append(self.robot.getDevice(self.JOINTS[i]))
            self.joint[i].setPosition(float('inf'))
            self.joint[i].setVelocity(0)

            self.pos_sensor.append(self.robot.getDevice(self.POS_SENSORS[i]))
            self.pos_sensor[i].enable(self.WEBOTS_BASIC_TIMESTEP)

        self.accel = self.robot.getDevice('Accelerometer')
        self.accel.enable(self.WEBOTS_BASIC_TIMESTEP)

        self.gyro = self.robot.getDevice('Gyro')
        self.gyro.enable(self.WEBOTS_BASIC_TIMESTEP)

        self.goal_position_sub = self.node.create_subscription(
            msg_type    = altair_interfaces.JointPosition,
            topic       = self.ROBOT_NAME + '/goal_position',
            callback    = self.goalPositionCallback,
            qos_profile = 1000
        )

        self.present_pos_pub = self.node.create_publisher(
            msg_type    = altair_interfaces.JointPosition,
            topic       = self.ROBOT_NAME + '/present_position',
            qos_profile = 1000
        )

        self.inertial_pub = self.node.create_publisher(
            msg_type    = altair_interfaces.Inertial,
            topic       = self.ROBOT_NAME + "/inertial",
            qos_profile = 1000
        )


    
    def setAsPositionControl(self) -> None:
        for i in range(self.JOINT_NUM):
            self.joint[i].setVelocity(self.joint[i].getMaxVelocity())
            self.joint[i].setPosition(self.pos_sensor[i].getValue())



    def setAsVelocityControl(self) -> None:
        for joint in self.joint:
            joint.setPosition(float('inf'))
            joint.setVelocity(0)



    def goalPositionCallback(self, msg:altair_interfaces.JointPosition) -> None:
        pass



    def sensorDataPublish(self) -> None:
        present_position_msg    = altair_interfaces.JointPosition()
        inertial_msg            = altair_interfaces.Inertial()

        for pos_sensor in self.pos_sensor:
            present_position_msg.val.append(2048 + int(pos_sensor.getValue()*651.8986469))
        
        inertial_msg.accel  = self.accel.getValues()
        inertial_msg.gyro   = self.gyro.getValues()
        inertial_msg.mag    = [0.0, 0.0, 0.0]

        self.present_pos_pub.publish(present_position_msg)
        self.inertial_pub.publish(inertial_msg)



    def step(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0)
        self.sensorDataPublish()