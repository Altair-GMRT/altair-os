const rclnodejs = require('rclnodejs')
const path      = require('path')
const fs        = require('fs')
const jsyaml    = require('js-yaml')
const config    = require('./config')


class RclGateway {

    constructor() {
        this.robot_config           = config.robot_config
        this.joint_config           = config.joint_config
        this.pose_studio_path       = config.pose_studio_path
        this.motion_sequencer_path  = config.motion_sequencer_path

        this.res_msg = {
            present_torque: null,
            present_position: null,
            present_velocity: null,
            joint_sensor: {
                load: null,
                voltage: null,
                temperature: null
            }
        }

        this.req_msg = {
            goal_torque: {val: null},
            goal_position: {val: null},
            goal_velocity: {val: null}
        }

        this.set_torque     = false
        this.set_position   = false
        this.set_velocity   = false
    }


    init() {
        rclnodejs.init().then(() => {
            const node = new rclnodejs.Node(`${this.robot_config.id}_app_server`)
            
            node.createSubscription(
                'altair_interfaces/msg/JointTorque',
                `${this.robot_config.id}/present_torque`,
                (msg) => {
                    this.res_msg.present_torque = msg.val
                }
            )

            node.createSubscription(
                'altair_interfaces/msg/JointPosition',
                `${this.robot_config.id}/present_position`,
                (msg) => {
                    this.res_msg.present_position = msg.val
                }
            )

            node.createSubscription(
                'altair_interfaces/msg/JointVelocity',
                `${this.robot_config.id}/present_velocity`,
                (msg) => {
                    this.res_msg.present_velocity = msg.val
                }
            )

            node.createSubscription(
                'altair_interfaces/msg/JointSensor',
                `${this.robot_config.id}/joint_sensor`,
                (msg) => {
                    this.res_msg.joint_sensor.load          = msg.load
                    this.res_msg.joint_sensor.voltage       = msg.voltage
                    this.res_msg.joint_sensor.temperature   = msg.temperature
                }
            )

            const goal_torque_pub = node.createPublisher(
                'altair_interfaces/msg/JointTorque',
                `${this.robot_config.id}/goal_torque`
            )

            const goal_position_pub = node.createPublisher(
                'altair_interfaces/msg/JointPosition',
                `${this.robot_config.id}/goal_position`
            )

            const goal_velocity_pub = node.createPublisher(
                'altair_interfaces/msg/JointVelocity',
                `${this.robot_config.id}/goal_velocity`
            )

            setInterval(() => {
                if(this.set_torque) {
                    this.set_torque = false
                    goal_torque_pub.publish(this.req_msg.goal_torque)
                }

                if(this.set_position) {
                    this.set_position = false
                    goal_position_pub.publish(this.req_msg.goal_position)
                }

                if(this.set_velocity) {
                    this.set_velocity = false
                }
            }, 50)
            
            node.spin()
        })
    }


    getStatus() {
        return this.res_msg
    }


    setTorque(values) {
        this.set_torque = true
        this.req_msg.goal_torque.val = values
    }


    setPosition(values) {
        this.set_position = true
        this.req_msg.goal_position.val = values
    }


    savePose(filename, values) {
        const yaml_str  = jsyaml.dump({val: values})
        const file_path = path.join(this.pose_studio_path, `${filename}.yaml`)
        fs.writeFileSync(file_path, yaml_str)
    }


    getSavedPoses() {
        const dir_path = this.pose_studio_path
        try {
            return {poses: fs.readdirSync(dir_path)}
        }
        catch(error) {
            return {poses: []}
        }
    }


    getPoseValue(filename) {
        const file_path = path.join(this.pose_studio_path, filename)
        try {
            return jsyaml.load(fs.readFileSync(file_path, 'utf8'))
        }
        catch(error) {
            return {val: []}
        }
    }


    saveMotion(filename, values) {
        const yaml_str  = jsyaml.dump({val: values})
        const file_path = path.join(this.motion_sequencer_path, `${filename}.yaml`)
        fs.writeFileSync(file_path, yaml_str)
    }


    getSavedMotions() {
        const dir_path = this.motion_sequencer_path
        try {
            return {motions: fs.readdirSync(dir_path)}
        }
        catch(error) {
            return {motions: []}
        }
    }


    getMotionValue(filename) {
        const file_path = path.join(this.motion_sequencer_path, filename)
        try {
            return jsyaml.load(fs.readFileSync(file_path, 'utf8'))
        }
        catch(error) {
            return {val: []}
        }
    }
}


module.exports = RclGateway