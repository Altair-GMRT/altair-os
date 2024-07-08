#include "arduino_data/arduino_data.h"


Arduino_Data::Arduino_Data(int argc, char** argv):
  init_argc_(argc),
  init_argv_(argv),
  data_ready(false),
  reconnect(false),
  broadcast(false),
  timeOut_ctr(0),
  dev_connected(false),
  readyToSend(false)
{

  if(!initialize())
    return;
}

Arduino_Data::~Arduino_Data()
{

}

bool Arduino_Data::initialize()
{
  RCLCPP_WARN("Initialize Arduino Controller");
  portName = "ttyACM0";
  baudRate = 1000000;
  waitTimeout = 500;
  broadcast = true;
  deviceCheck();
  std::string default_path;
  default_path = "../../config/GlobalConfig.yaml";
  loadButton(default_path);
  serial(portName);
  serial.SetBaudRate(LibSerial::BaudRate::BAUD_1000000);
  serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial.SetParity(LibSerial::Parity::PARITY_NONE);
  serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

  return true;

}

void Arduino_Data::loadButton(const std::string path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    return;
  }

  // parse button_setting
  YAML::Node button_mode = doc["button_setting"];
  for (YAML::iterator yaml_it = button_mode.begin(); yaml_it != button_mode.end(); ++yaml_it)
  {
    int index;
    std::string mode;
    index = yaml_it->second.as<int>();
    mode = yaml_it->first.as<std::string>();
    button_mapping.insert(std::make_pair(index, mode));
  }

  YAML::Node ardu = doc["Arduino_Data"];

  offset = ardu["offset"].as<double>();

}

void Arduino_Data::debugDevice()
{
  RCLCPP_WARN("Baud Rate: %d", baudRate);
  RCLCPP_WARN("Port Name: %s", portName);
}

void Arduino_Data::deviceCheck()
{
  // std::vector<LibSerial::SerialPortInfo> ports = LibSerial::SerialPort::GetAvailableSerialPorts();
  // for(auto p : ports)
  // {
  //   if(portName == p)
  //   {
  //     RCLCPP_WARN("Found Port: %s" , p.c_str());
  //     devicedata = p;
  //     debugDevice();
  //     dev_connected = true;
  //     break;
  //   }
  // }

  mutex.lock();

  if (currentPortName != portName) {
    currentPortName = portName;
    currentPortNameChanged = true;
    dev_connected = false;
  }

  mutex.unlock();
}

void Arduino_Data::run(std::promise<void>& quitSignal)
{

  while (rclcpp::ok())
  {
    deviceLoop();
    if(data_ready)
      publishData();
    rclcpp::spinOnce(ros_node);
  }

  RCLCPP_WARN("Ros shutdown, proceeding to close the port");
  // Signal that the task is finished
  quitSignal.set_value();
  std::exit(0);
}

void Arduino_Data::deviceLoop()
{
  if (currentPortNameChanged || reconnect)
  {
    if(reconnect)
      RCLCPP_WARN("Arduino Reconnecting");
    serial.Close(portName);
    serial(portName);
    serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial.SetParity(LibSerial::Parity::PARITY_NONE);
    serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    devicedata = portName;
    currentPortNameChanged = false;
    debugDevice();
    }

    try
    {
        serial.Open(portName);
    }
    catch (const OpenFailed&)
    {
        RCLCPP_ERROR("Arduino Error open device %s", portName.c_str());
        dev_connected = false;
        return EXIT_FAILURE;
    }
    dev_connected = true;
    reconnect = false;
    timeOut_ctr = 0;
    RCLCPP_WARN("Arduino Ready to Read");
  }

  if (serial.IsDataAvailable()) {
    reconnect = false;
    timeOut_ctr = 0;
    // read request
    LibSerial::DataBuffer requestData;
    serial.read(requestData, 0, 500);
    for(auto data: requestData)
    {
      switch (arduino_state)
      {
      case Ardu_Start_U :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'G')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          roll = temp_data
          
          arduino_state = Ardu_Start_G;
          temp_data.clear();
        }
        break;
      }

      case Ardu_Start_G :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'M')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          pitch = temp_data;
          arduino_state = Ardu_Start_M;
          temp_data.clear();
        }
        break;

      }

      case Ardu_Start_M :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'D')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          yaw = temp_data;
          arduino_state = Ardu_Start_D;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_D :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'I')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          gyroRoll = temp_data;
          arduino_state = Ardu_Start_I;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_I :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'Y')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          gyroPitch = temp_data;
          arduino_state = Ardu_Start_Y;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_Y :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'P')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          gyroYaw = temp_data;
          arduino_state = Ardu_Start_P;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_P :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'C')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          accelX = temp_data;
          arduino_state = Ardu_Start_C;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_C :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'T')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          accelY = temp_data;
          arduino_state = Ardu_Start_T;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_T :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'B')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          accelZ = temp_data;
          arduino_state = Ardu_Start_B;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_B :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'A')
        {
          temp_data = std::regex_replace(temp_data, regexPattern, "");
          QString button_ = temp_data;
          button = button_.toInt();
          arduino_state = Ardu_Start_U;
          data_ready = true;
          temp_data.clear();
        }
        break;
      }
    }


  } else {
    RCLCPP_WARN("Timeout");
    timeOut_ctr++;
    if(timeOut_ctr>100)
    {
      reconnect = true;
      dev_connected = false;
    }

  }

}

void Arduino_Data::publishData()
{
  double roll_ = -roll.toDouble() * DEGREE2RADIAN; double pitch_ = pitch.toDouble() * DEGREE2RADIAN; double yaw_ = -yaw.toDouble() * DEGREE2RADIAN;
  roll.clear(); pitch.clear(); yaw.clear();
  double gyroRoll_ = gyroRoll.toDouble() * DEGREE2RADIAN; double gyroPitch_ = -gyroPitch.toDouble() * DEGREE2RADIAN; double gyroYaw_ = gyroYaw.toDouble() * DEGREE2RADIAN;
  gyroRoll.clear(); gyroPitch.clear(); gyroYaw.clear();
  double accelX_ = accelX.toDouble(); double accelY_ = accelY.toDouble(); double accelZ_ = accelZ.toDouble();

  if(!dev_connected)
  {
    roll_ = pitch_ = yaw_ = gyroRoll_ = gyroPitch_ = gyroYaw_ = 0;
  }


  Eigen::Quaterniond quaternion = robotis_framework::convertRPYToQuaternion(roll_, pitch_, yaw_);
  Eigen::Vector3 gyro(gyroRoll_, gyroPitch_, gyroYaw_);
  Eigen::Vector3 accel(accelX_, accelY_, accelZ_);
  sensor_msgs::msg::Imu imu;

  tf::StampedTransform transform;
  tf::TransformBroadcaster br;

  tf::quaternionEigenToMsg(quaternion, imu.orientation);
  tf::vectorEigenToMsg(gyro, imu.angular_velocity);
  tf::vectorEigenToMsg(accel, imu.linear_acceleration);
  imu.header.stamp = rclcpp::Time::now();
  imu.header.frame_id = "imu_link";

  imu_pub->publish(imu);

  std_msgs::msg::String button_str;

  if(button != prev_button)
  {
    std::map<int, std::string>::iterator iter = button_mapping.find(button);
    if (iter != button_mapping.end())
    {
      std::string data_ = iter->second;
      button_str.data = data_;
    }
    else button_str.data = std::string("None");

  readyToSend = true;

  }

  prev_button = button;

  if(readyToSend)
{
readyToSend = false;
button_pub->publish(button_str);
}
  if(!dev_connected || reconnect)
    button_str.data = std::string("None");

  //check bentuk imu di rviz
  if(broadcast)
  {
    transform.frame_id_= "base_link";
    transform.child_frame_id_ = "imu_link";
    transform.stamp_ = rclcpp::Time::now();
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
    transform.setRotation(q);
    br.sendTransform(transform);
  }


}
