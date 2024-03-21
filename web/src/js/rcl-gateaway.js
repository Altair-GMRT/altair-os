const rclnodejs = require('rclnodejs')
const config = require('./config')


class RclGateAway {

    constructor() {
        this.robot_config = config.robot_config
        this.joint_config = config.joint_config
    }


    rclInit() {
        rclnodejs.init().then(() => {
            const node = new rclnodejs.Node(`${this.robot_config.id}`)
            node.spin()
        })
    }
}


module.exports = RclGateAway