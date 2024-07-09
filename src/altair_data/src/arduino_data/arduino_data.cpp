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
  std::string default_path;
  default_path = "../../config/GlobalConfig.yaml";
  loadButton(default_path);
  try {
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  }
  catch {
    RCLCPP_ERROR("Error Opening Port");
    return false;
  }

  // CONFIGURE PORT
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd, &tty) != 0) {
      cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
      return false;
  }

  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);

  tty.c_cflag &= ~PARENB; // Make 8n1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS; // No flow control
  tty.c_cc[VMIN] = 1; // Read at least 1 character
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_cflag |= CREAD | CLOCAL; // Turn on the READ & ignore ctrl lines

  cfmakeraw(&tty);

  tcflush(fd, TCIFLUSH);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      cerr << "Error " << errno << " from tcsetattr" << endl;
  }

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

void Arduino_Data::configure_port(int fd)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return;
    }

    cfsetospeed(&tty, B1000000);
    cfsetispeed(&tty, B1000000);

    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS; // No flow control
    tty.c_cc[VMIN] = 1; // Read at least 1 character
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_cflag |= CREAD | CLOCAL; // Turn on the READ & ignore ctrl lines

    cfmakeraw(&tty);

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Error " << errno << " from tcsetattr" << endl;
    }
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
  // close(fd);

  RCLCPP_WARN("Ros shutdown, proceeding to close the port");
  // Signal that the task is finished
  // quitSignal.set_value();
  // std::exit(0);
}

void Arduino_Data::deviceLoop()
{
  // fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
  // if (fd < 0) {
  //   cerr << "Error " << errno << " opening " << portName << ": " << strerror(errno) << endl;
  //   return;
  // }
  if(reconnect) {
    close(fd);
    Arduino_Data::initialize();
  }
  
  std::ifstream serial(portName);

  // removed while(getline(...)) {}
  std::getline(serial, read_buffer)
  std::istringstream iss(read_buffer);
  char type;
  double value;
  dev_connected = true;

  if (iss >> type >> value) {
    switch (type) {
      case 'U':
          // std::cout << "Type U: " << value << '\n';
          roll = value;
          data_ready = false;
          break;
      case 'G':
          // std::cout << "Type G: " << value << '\n';
          pitch = value;
          data_ready = false;
          break;
      case 'M':
          // std::cout << "Type M: " << value << '\n';
          yaw = value;
          data_ready = false;
          break;
      case 'D':
          // std::cout << "Type D: " << value << '\n';
          gyroRoll = value;
          data_ready = false;
          break;
      case 'I':
          // std::cout << "Type I: " << value << '\n';
          gyroPitch = value;
          data_ready = false;
          break;
      case 'Y':
          // std::cout << "Type Y: " << value << '\n';
          gyroYaw = value;
          data_ready = false;
          break;
      case 'P':
          // std::cout << "Type P: " << value << '\n';
          accelX = value;
          data_ready = false;
          break;
      case 'C':
          // std::cout << "Type C: " << value << '\n';
          accelY = value;
          data_ready = false;
          break;
      case 'T':
          // std::cout << "Type T: " << value << '\n';
          accelZ = value;
          data_ready = true;
          break;
      case 'B':
          // std::cout << "Type B: " << value << '\n';
          button = value;
          data_ready = true;
          break;
    }
  }
  else {
    // std::cerr << "Failed to parse line: " << line << '\n';
    RCLCPP_ERROR("Failed to Parse Data");
    reconnect = true;
    dev_connected = false;
    return;
  }
}

void Arduino_Data::publishData()
{
  double roll_ = -roll * DEGREE2RADIAN; double pitch_ = pitch * DEGREE2RADIAN; double yaw_ = -yaw * DEGREE2RADIAN;
  double gyroRoll_ = gyroRoll * DEGREE2RADIAN; double gyroPitch_ = -gyroPitch * DEGREE2RADIAN; double gyroYaw_ = gyroYaw * DEGREE2RADIAN;
  double accelX_ = accelX; double accelY_ = accelY; double accelZ_ = accelZ;

  if(!dev_connected)
  {
    roll_ = pitch_ = yaw_ = gyroRoll_ = gyroPitch_ = gyroYaw_ = 0;
  }


  Eigen::Quaterniond quaternion = robotis_framework::convertRPYToQuaternion(roll_, pitch_, yaw_);
  Eigen::Vector3 gyro(gyroRoll_, gyroPitch_, gyroYaw_);
  Eigen::Vector3 accel(accelX_, accelY_, accelZ_);
  sensor_msgs::msg::Imu imu;

  // DEPRECATED CODES
  // tf2::StampedTransform transform;
  // tf2::TransformBroadcaster br;
  // tf2::quaternionEigenToMsg(quaternion, imu.orientation);
  // tf2::vectorEigenToMsg(gyro, imu.angular_velocity);
  // tf2::vectorEigenToMsg(accel, imu.linear_acceleration);

  geometry_msgs::msg::TransformStamped transform;
  tf2_ros::TransformBroadcaster br;
  tf2::convert(quaternion, imu_msg.orientation);
  tf2::convert(gyro, imu_msg.angular_velocity);
  tf2::convert(accel, imu_msg.linear_acceleration);


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
    transform.setOrigin(tf2::Vector3(0,0,0));
    tf2::Quaternion q = tf2::createQuaternionFromRPY(roll_, pitch_, yaw_);
    transform.setRotation(q);
    br.sendTransform(transform);
  }


}
