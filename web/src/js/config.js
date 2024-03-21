const fs        = require('fs')
const path      = require('path')
const jsyaml    = require('js-yaml')

const ALTAIR_OS_PATH            = '/altair-os'
const ALTAIR_DATA_PATH          = path.join(ALTAIR_OS_PATH, 'src/altair_data')
const ROBOT_CONFIG_YAML_PATH    = path.join(ALTAIR_DATA_PATH, 'config/robot_config.yaml')
const JOINT_CONFIG_YAML_PATH    = path.join(ALTAIR_DATA_PATH, 'config/joint_config.yaml')

const ROBOT_CONFIG  = jsyaml.load(fs.readFileSync(ROBOT_CONFIG_YAML_PATH, 'utf8'))
const JOINT_CONFIG  = jsyaml.load(fs.readFileSync(JOINT_CONFIG_YAML_PATH, 'utf8'))

module.exports.robot_config   = ROBOT_CONFIG
module.exports.joint_config   = JOINT_CONFIG