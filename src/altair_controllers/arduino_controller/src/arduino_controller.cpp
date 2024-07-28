#include "arduino_controller/arduino_controller.h"

Arduino_Data::~Arduino_Data() {
}

void Arduino_Data::configure_port(int fd) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd, &tty) != 0) {
    cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    return;
  }

  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);

  tty.c_cflag &= ~PARENB;  // Make 8n1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS;  // No flow control
  tty.c_cc[VMIN] = 1;       // Read at least 1 character
  tty.c_cc[VTIME] = 5;      // 0.5 seconds read timeout

  tty.c_cflag |= CREAD | CLOCAL;  // Turn on the READ & ignore ctrl lines

  cfmakeraw(&tty);

  tcflush(fd, TCIFLUSH);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    cerr << "Error " << errno << " from tcsetattr" << endl;
  }

  // add low latency
  struct serial_struct serinfo;
  if (ioctl(fd, TIOCGSERIAL, &serinfo) != 0) {
    std::cerr << "Error " << errno << " from TIOCGSERIAL: " << strerror(errno) << std::endl;
    return;
  }

  serinfo.flags |= ASYNC_LOW_LATENCY;

  if (ioctl(fd, TIOCSSERIAL, &serinfo) != 0) {
    std::cerr << "Error " << errno << " from TIOCSSERIAL: " << strerror(errno) << std::endl;
    return;
  }
}

bool Arduino_Data::init() {
  RCLCPP_WARN(rclcpp::get_logger("arduino_data_node"), "Initialize Arduino Controller");
  portName = "/dev/ttyUSB0";
  baudRate = 9600;
  waitTimeout = 500;
  broadcast = true;
  std::string default_path;
  default_path = "../../config/GlobalConfig.yaml";
  // loadButton(default_path);
  try {
    fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino_data_node"), "Error Opening Port");
    return false;
  }

  // CONFIGURE PORT
  Arduino_Data::configure_port(fd);

  return true;
}

void Arduino_Data::read() {
  if (reconnect) {
    close(fd);
    if (Arduino_Data::init()) {
      reconnect = false;
    }
  }
  std::ifstream serial(portName);

  getline(serial, read_buffer);

  istringstream iss(read_buffer);

  if (data_ready) {
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_pub->publish(imu_msg);
  }

  if (iss >> type >> value && fd != -1) {
    switch (type) {
      case 'U':
        cout << "Type U: " << value << '\n';
        roll = -1 * value * DEGREE2RADIAN;
        imu_msg.orientation.x = roll;
        data_ready = false;
        break;
      case 'G':
        cout << "Type G: " << value << '\n';
        pitch = value * DEGREE2RADIAN;
        imu_msg.orientation.y = pitch;
        data_ready = false;
        break;
      case 'M':
        cout << "Type M: " << value << '\n';
        yaw = -1 * value * DEGREE2RADIAN;
        imu_msg.orientation.z = yaw;
        data_ready = false;
        break;
      case 'D':
        cout << "Type D: " << value << '\n';
        gyroRoll = value * DEGREE2RADIAN;
        imu_msg.angular_velocity.x = gyroRoll;
        data_ready = false;
        break;
      case 'I':
        cout << "Type I: " << value << '\n';
        gyroPitch = -1 * value * DEGREE2RADIAN;
        imu_msg.angular_velocity.y = gyroPitch;
        data_ready = false;
        break;
      case 'Y':
        cout << "Type Y: " << value << '\n';
        gyroYaw = value * DEGREE2RADIAN;
        imu_msg.angular_velocity.z = gyroYaw;
        data_ready = false;
        break;
      case 'P':
        cout << "Type P: " << value << '\n';
        accelX = value;
        imu_msg.linear_acceleration.x = accelX;
        data_ready = false;
        break;
      case 'C':
        cout << "Type C: " << value << '\n';
        accelY = value;
        imu_msg.linear_acceleration.y = accelY;
        data_ready = false;
        break;
      case 'T':
        cout << "Type T: " << value << '\n';
        accelZ = value;
        imu_msg.linear_acceleration.z = accelZ;
        data_ready = true;
        break;
      case 'B':
        cout << "Type B: " << value << '\n';
        button_msg.data = (char)value;
        button_pub->publish(button_msg);
        // data_ready = true;
        break;
    }
  } 
  else {
    reconnect = true;
    RCLCPP_ERROR(rclcpp::get_logger("arduino_data_node"),
                 "Error Parsing Data. Trying to reconnect Now... \nIf the error persist, try to reconnect USB or "
                 "restart arduino_data_node");
  }
}
