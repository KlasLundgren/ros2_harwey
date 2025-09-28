#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

class ImuDataPub : public rclcpp::Node
{
public:
  ImuDataPub()
  : Node("imu_data_pub"), serial_fd_(-1)
  {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    if (!init_serial_port()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
      return;
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&ImuDataPub::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "IMU Data Publisher initialized");
  }

  ~ImuDataPub()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  bool init_serial_port()
  {
    serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port /dev/ttyUSB0");
      return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error getting serial port attributes");
      close(serial_fd_);
      return false;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes");
      close(serial_fd_);
      return false;
    }

    return true;
  }

  void timer_callback()
  {
    read_serial_data();
    process_buffer();
  }

  void read_serial_data()
  {
    if (serial_fd_ < 0) return;

    int bytes_available = 0;
    ioctl(serial_fd_, FIONREAD, &bytes_available);

    if (bytes_available > 0) {
      std::vector<uint8_t> temp_buffer(bytes_available);
      int bytes_read = read(serial_fd_, temp_buffer.data(), bytes_available);

      if (bytes_read > 0) {
        buffer_.insert(buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + bytes_read);
      }
    }
  }

  void process_buffer()
  {
    while (buffer_.size() >= 11) {
      auto start_it = std::find(buffer_.begin(), buffer_.end(), 0x55);
      if (start_it == buffer_.end()) {
        buffer_.clear();
        break;
      }

      if (start_it != buffer_.begin()) {
        buffer_.erase(buffer_.begin(), start_it);
      }

      if (buffer_.size() >= 11) {
        std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 11);
        buffer_.erase(buffer_.begin(), buffer_.begin() + 11);

        parse_packet(packet);
      } else {
        break;
      }
    }
  }

  void parse_packet(const std::vector<uint8_t>& packet)
  {
    if (packet[0] != 0x55) return;

    uint8_t data_type = packet[1];
    int16_t val1 = (packet[3] << 8) | packet[2];
    int16_t val2 = (packet[5] << 8) | packet[4];
    int16_t val3 = (packet[7] << 8) | packet[6];

    if (data_type == 0x51) {
      accel_x_ = val1 / 32768.0 * 16.0 * 9.81;
      accel_y_ = val2 / 32768.0 * 16.0 * 9.81;
      accel_z_ = val3 / 32768.0 * 16.0 * 9.81;
      has_accel_ = true;
    }
    else if (data_type == 0x52) {
      gyro_x_ = val1 / 32768.0 * 2000.0 * M_PI / 180.0;
      gyro_y_ = val2 / 32768.0 * 2000.0 * M_PI / 180.0;
      gyro_z_ = val3 / 32768.0 * 2000.0 * M_PI / 180.0;
      has_gyro_ = true;
    }
    else if (data_type == 0x53) {
      roll_ = val1 / 32768.0 * 180.0 * M_PI / 180.0;
      pitch_ = val2 / 32768.0 * 180.0 * M_PI / 180.0;
      yaw_ = val3 / 32768.0 * 180.0 * M_PI / 180.0;
      has_orientation_ = true;
    }

    if (has_accel_ && has_gyro_ && has_orientation_) {
      publish_imu_data();
      has_accel_ = has_gyro_ = has_orientation_ = false;
    }
  }

  void publish_imu_data()
  {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = accel_x_;
    imu_msg.linear_acceleration.y = accel_y_;
    imu_msg.linear_acceleration.z = accel_z_;

    imu_msg.angular_velocity.x = gyro_x_;
    imu_msg.angular_velocity.y = gyro_y_;
    imu_msg.angular_velocity.z = gyro_z_;

    double cy = cos(yaw_ * 0.5);
    double sy = sin(yaw_ * 0.5);
    double cp = cos(pitch_ * 0.5);
    double sp = sin(pitch_ * 0.5);
    double cr = cos(roll_ * 0.5);
    double sr = sin(roll_ * 0.5);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

    for (int i = 0; i < 9; i++) {
      imu_msg.linear_acceleration_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.orientation_covariance[i] = 0.0;
    }
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;

    imu_publisher_->publish(imu_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  int serial_fd_;
  std::vector<uint8_t> buffer_;

  double accel_x_, accel_y_, accel_z_;
  double gyro_x_, gyro_y_, gyro_z_;
  double roll_, pitch_, yaw_;

  bool has_accel_ = false;
  bool has_gyro_ = false;
  bool has_orientation_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDataPub>());
  rclcpp::shutdown();
  return 0;
}