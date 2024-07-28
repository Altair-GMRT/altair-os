#include "dynamixel_controller/dynamixel_controller.h"

namespace altair_hardware {
using std::placeholders::_1;

Dynamixel_Controller::Dynamixel_Controller(rclcpp::Node::SharedPtr nh) {
  nh_ = nh;
  DEBUG_PRINT = true;

  driver_ = std::make_shared<DynamixelWorkbench>();
  robot_ = std::make_shared<Robot>();

  // RobotisController_ = std::make_shared<RobotisController>();
}

bool Dynamixel_Controller::init() {
  // Init subscriber / publisher
  set_torque_sub_ = nh_->create_subscription<std_msgs::msg::Bool>(
      "/set_torque", 1, std::bind(&Dynamixel_Controller::setTorqueCb, this, _1));

  sub_command_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      "/DynamixelController/command", 1, std::bind(&Dynamixel_Controller::JointGoalCb, this, std::placeholders::_1));
  joint_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  temperature_pub_ = nh_->create_publisher<std_msgs::msg::Int32MultiArray>("/servo_temperature", 10);

  // torqueless_mode_ = nh_->get_parameter("torqueless_mode").as_bool();

  // init merged vectors for controller
  joint_count_ = joint_names_.size();
  current_position_.resize(joint_count_, 0);
  current_velocity_.resize(joint_count_, 0);
  current_effort_.resize(joint_count_, 0);
  current_pwm_.resize(joint_count_, 0);
  current_input_voltage_.resize(joint_count_, 0);
  current_temperature_.resize(joint_count_, 0);
  current_error_.resize(joint_count_, 0);
  goal_position_.resize(joint_count_, 0);
  goal_velocity_.resize(joint_count_, 0);
  goal_acceleration_.resize(joint_count_, 0);
  goal_effort_.resize(joint_count_, 0);
  goal_torque_individual_.resize(joint_count_, 1);

  // create mapping for faster computation later
  for (unsigned int i = 0; i < joint_count_; i++) {
    joint_map_[joint_names_[i]] = i;
  }

  joint_state_msg_ = sensor_msgs::msg::JointState();
  joint_state_msg_.name = joint_names_;
  pwm_msg_ = sensor_msgs::msg::JointState();
  pwm_msg_.name = joint_names_;

  // dynamixel initialization, prepare for sync write and sync read
  if (!Dynamixel_Controller::setupDevice()) {
    return false;
  }

  RCLCPP_INFO(nh_->get_logger(), "Hardware interface init finished.");
  return true;
}

bool Dynamixel_Controller::setupDevice() {
  port_name = "/dev/ttyUSB0";
  baudrate = 1000000;
  if (!driver_->init(port_name, baudrate)) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to open port or set baud rate");
    return false;
  }
  Dynamixel_Controller::ping();

  // disable while debugging as debugger can't find the file on my workspace
  Dynamixel_Controller::loadRobotConfig();
  Dynamixel_Controller::loadOffsetConfig();
  Dynamixel_Controller::loadDynamixelConfig();
  // sync write init
  driver_->addSyncWriteHandler(10, "Goal_Position");
  driver_->addSyncWriteHandler(10, "Position_P_Gain");
  driver_->addSyncWriteHandler(10, "Position_I_Gain");
  driver_->addSyncWriteHandler(10, "Position_D_Gain");

  // bulk read init
  driver_->addSyncReadHandler(10, "Present_Position");
  driver_->addSyncReadHandler(10, "Present_Temperature");
  return true;
}

// TODO: Use Dynamixel Workbench
bool Dynamixel_Controller::loadDynamixelConfig() {
  std::string a = ament_index_cpp::get_package_share_directory("dynamixel_controller");
  std::string dxl_config = a + "/config/dxl_init.yaml";
  int i = 0;
  YAML::Node config_node;
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(dxl_config);
    // iterate joint name in dxl_init.yaml
    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++) {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];

      // interate child component of joint_node
      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++) {
        // get the item_name and value of joint_node
        std::string item_name = it_joint->first.as<std::string>();
        uint32_t value = it_joint->second.as<uint32_t>();
        std::cout << "Item name: " << item_name << std::endl;
        i = 0;
        for (auto& key : robot_->joint_map) {
          i++;
          
          // find the mathing pair of joint_name from offset.yaml to its id
          if (key.second.name == joint_name) {
            if (DEBUG_PRINT)
              RCLCPP_INFO(nh_->get_logger(), "ID: %s JOINTNAME: %s ITEM_NAME: %s VALUE: %s", i, joint_name, item_name,
                          value);
            // write the value to the register
            driver_->itemWrite(i, item_name.c_str(), value);
            break;
          }
        }
        
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(nh_->get_logger(), "Dynamixel Init file not found.");
  }
  return true;
}

bool Dynamixel_Controller::loadOffsetConfig() {
  // find the config file path
  std::string package_name = "dynamixel_controller";
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string file_path = package_share_directory + "/config/offset.yaml";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(file_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load Offset Config");
    return false;
  }
  int i = 0;
  YAML::Node offset_node = config_node["offset"];
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); ++it) {
    i = 0;
    for (auto& key : robot_->joint_map) {
      i++;
      // find the mathing pair of joint_name from offset.yaml to robot_->joint_map index key
      if (key.second.name == it->first.as<std::string>()) {
        robot_->joint_map[i].joint_offset =  it->second.as<double>();
        break;
        // std::cout << robot_->joint_map[i].name << ": " << robot_->joint_map[i].joint_offset << std::endl;
      }
    }
  }
  return true;
}

bool Dynamixel_Controller::loadRobotConfig() {
  // find the config file path
  std::string package_name = "dynamixel_controller";
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
  // std::cout << package_share_directory << std::endl;
  std::string file_path = package_share_directory + "/config/robot.yaml";
  // std::cout << file_path << std::endl;
  YAML::Node config_node;

  try {
    config_node = YAML::LoadFile(file_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load Robot Config");
    return false;
  }

  // parse portname, baudrate, protocol, and default joint
  robot_->port_name = config_node["PORTNAME"].as<std::string>();
  robot_->baudrate = config_node["BAUDRATE"].as<int>();
  robot_->protocol = config_node["PROTOCOL"].as<double>();
  robot_->default_joint = config_node["DEFAULT_JOINT"].as<std::string>();

  // parse every device joint_name, id, & info
  YAML::Node devices_node = config_node["DEVICE"];
  for (YAML::const_iterator it = devices_node.begin(); it != devices_node.end(); ++it) {
    int key = it->first.as<uint8_t>();
    YAML::Node device_node = it->second;

    Servo device;
    device.name = device_node["NAME"].as<std::string>();
    // std::cout << device.name << std::endl;
    device.id = device_node["ID"].as<uint8_t>();
    device.model = device_node["MODEL"].as<std::string>();

    robot_->joint_map[key] = device;
  }

  return true;
}

void Dynamixel_Controller::JointGoalCb(const sensor_msgs::msg::JointState& command_msg) {
  for (uint8_t dxl_id = 1; dxl_id <= 20; dxl_id++) {
    goal_position_[dxl_id] = (uint32_t)command_msg.position[dxl_id];
  }
}

void Dynamixel_Controller::setTorqueCb(std_msgs::msg::Bool::SharedPtr enabled) {
  if (enabled->data == true) {
    for (uint8_t dxl_id = 0; dxl_id < 20; dxl_id++) {
      if (!driver_->torqueOn(dxl_id)) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to enable torque for Dynamixel ID %d", dxl_id);
      } else {
        RCLCPP_INFO(nh_->get_logger(), "Dynamixel ID %d has been successfully connected", dxl_id);
      }
    }
  }
}

void Dynamixel_Controller::read() {
  // Present_Position
  if (!driver_->syncRead(0, &log)) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to syncRead");
  }
  driver_->getSyncReadData(0, data_sync_read_positions_, &log);
  // Present_Temperature
  driver_->syncRead(1, &log);
  driver_->getSyncReadData(1, data_sync_read_temperature_, &log);

  for (uint8_t dxl_id = 1; dxl_id <= 20; dxl_id++) {
    driver_->convertValue2Radian(dxl_id, data_sync_read_positions_[dxl_id]);
    joint_state_msg_.position[dxl_id] = data_sync_read_positions_[dxl_id];
    temperature_msg_.data[dxl_id] = data_sync_read_temperature_[dxl_id];
  }

  joint_pub_->publish(joint_state_msg_);
  temperature_pub_->publish(temperature_msg_);
}

void Dynamixel_Controller::write() {
  for (uint8_t dxl_id = 1; dxl_id <= 20; dxl_id++) {
    uint32_t temp_goal_positon = goal_position_[dxl_id] + robot_->joint_map[dxl_id].joint_offset;
    sync_write_goal_position_[dxl_id] = driver_->convertRadian2Value(dxl_id, temp_goal_positon);
  }
  driver_->syncWrite(0, sync_write_goal_position_, &log);
  // TODO: Write PID to the Servos
}

bool Dynamixel_Controller::ping() {
  for (size_t dxl_id = 0; dxl_id <= 20; dxl_id++) {
    if (!driver_->ping(dxl_id)) {
      printf("Failed to ping servo ID %d\n", dxl_id);
    }
  }
  return true;
}

// ROBOTIS_OP FRAMEWORK PROPRIETARY
// TODO: Scrap the following functions and use dynamixel workbench
/**
int RobotisController::read(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data,
                            uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->readTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::readCtrlItem(const std::string joint_name, const std::string item_name, uint32_t* data,
                                    uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem* item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  switch (item->data_length_) {
    case 1: {
      uint8_t read_data = 0;
      result = pkt_handler->read1ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 2: {
      uint16_t read_data = 0;
      result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 4: {
      uint32_t read_data = 0;
      result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    default:
      break;
  }
  return result;
}

int RobotisController::read1Byte(const std::string joint_name, uint16_t address, uint8_t* data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->read1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read2Byte(const std::string joint_name, uint16_t address, uint16_t* data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->read2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::read4Byte(const std::string joint_name, uint16_t address, uint32_t* data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->read4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data,
                             uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->writeTxRx(port_handler, dxl->id_, address, length, data, error);
}

int RobotisController::writeCtrlItem(const std::string joint_name, const std::string item_name, uint32_t data,
                                     uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  ControlTableItem* item = dxl->ctrl_table_[item_name];
  if (item == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  uint8_t* write_data = new uint8_t[item->data_length_];
  if (item->data_length_ == 1) {
    write_data[0] = (uint8_t)data;
    result = pkt_handler->write1ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  } else if (item->data_length_ == 2) {
    write_data[0] = DXL_LOBYTE((uint16_t)data);
    write_data[1] = DXL_HIBYTE((uint16_t)data);
    result = pkt_handler->write2ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  } else if (item->data_length_ == 4) {
    write_data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)data));
    write_data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)data));
    write_data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)data));
    write_data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)data));
    result = pkt_handler->write4ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  delete[] write_data;
  return result;
}

int RobotisController::write1Byte(const std::string joint_name, uint16_t address, uint8_t data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->write1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write2Byte(const std::string joint_name, uint16_t address, uint16_t data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->write2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::write4Byte(const std::string joint_name, uint16_t address, uint32_t data, uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->write4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int RobotisController::regWrite(const std::string joint_name, uint16_t address, uint16_t length, uint8_t* data,
                                uint8_t* error) {
  // if (isTimerStopped() == false)
  //   return COMM_PORT_BUSY;

  Dynamixel* dxl = RobotObject->dxls_[joint_name];
  if (dxl == NULL)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = RobotObject->ports_[dxl->port_name_];

  return pkt_handler->regWriteTxRx(port_handler, dxl->id_, address, length, data, error);
}
**/
}  // namespace altair_hardware
