#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

class SensorFusionNode : public rclcpp::Node
{
public:
  SensorFusionNode()
  : Node("sensor_fusion_node"),
    position_(Eigen::Vector3d::Zero()),
    velocity_(Eigen::Vector3d::Zero()),
    last_imu_time_(this->now())
  {
    // Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", 10,
      std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SensorFusionNode::lidar_callback, this, std::placeholders::_1));

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fused_pointcloud", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Calculate time delta
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_imu_time_).seconds();
    last_imu_time_ = current_time;

    // Skip first iteration or if dt is too large (indicates first message or reset)
    if (dt <= 0.0 || dt > 1.0) {
      current_orientation_ = msg->orientation;
      return;
    }

    // Store current IMU data
    current_imu_ = *msg;
    current_orientation_ = msg->orientation;

    // Convert quaternion to rotation matrix for transforming acceleration
    Eigen::Quaterniond q(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z
    );
    Eigen::Matrix3d rotation = q.toRotationMatrix();

    // Transform acceleration from body frame to world frame
    Eigen::Vector3d accel_body(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z
    );
    Eigen::Vector3d accel_world = rotation * accel_body;

    // Remove gravity (assuming z-up world frame)
    accel_world.z() -= 9.81;

    // Simple integration for velocity and position (dead reckoning)
    // Note: This will drift over time without additional corrections
    velocity_ += accel_world * dt;
    position_ += velocity_ * dt;

    // Publish odometry
    publish_odometry(current_time);
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan to PointCloud2
    pcl::PointCloud<pcl::PointXYZ> cloud;

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];

      // Filter invalid points
      if (range >= msg->range_min && range <= msg->range_max) {
        pcl::PointXYZ point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;  // LaserScan is 2D
        cloud.points.push_back(point);
      }

      angle += msg->angle_increment;
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    // Convert to PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = msg->header;

    // Publish pointcloud
    pointcloud_pub_->publish(cloud_msg);
  }

  void publish_odometry(const rclcpp::Time& current_time)
  {
    // Create odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Set position
    odom_msg.pose.pose.position.x = position_.x();
    odom_msg.pose.pose.position.y = position_.y();
    odom_msg.pose.pose.position.z = position_.z();

    // Set orientation from IMU
    odom_msg.pose.pose.orientation = current_orientation_;

    // Set velocity
    odom_msg.twist.twist.linear.x = velocity_.x();
    odom_msg.twist.twist.linear.y = velocity_.y();
    odom_msg.twist.twist.linear.z = velocity_.z();

    // Set angular velocity from IMU
    odom_msg.twist.twist.angular.x = current_imu_.angular_velocity.x;
    odom_msg.twist.twist.angular.y = current_imu_.angular_velocity.y;
    odom_msg.twist.twist.angular.z = current_imu_.angular_velocity.z;

    // Set covariances (relatively high due to IMU drift)
    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[14] = 0.1;  // z
    odom_msg.pose.covariance[21] = 0.05; // rot x
    odom_msg.pose.covariance[28] = 0.05; // rot y
    odom_msg.pose.covariance[35] = 0.05; // rot z

    odom_msg.twist.covariance[0] = 0.1;
    odom_msg.twist.covariance[7] = 0.1;
    odom_msg.twist.covariance[14] = 0.1;
    odom_msg.twist.covariance[21] = 0.05;
    odom_msg.twist.covariance[28] = 0.05;
    odom_msg.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom_msg);

    // Broadcast transform from odom to base_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = position_.x();
    transform.transform.translation.y = position_.y();
    transform.transform.translation.z = position_.z();
    transform.transform.rotation = current_orientation_;

    tf_broadcaster_->sendTransform(transform);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State variables
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  rclcpp::Time last_imu_time_;
  sensor_msgs::msg::Imu current_imu_;
  geometry_msgs::msg::Quaternion current_orientation_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}
