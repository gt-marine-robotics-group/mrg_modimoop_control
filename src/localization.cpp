#include <cmath>
#include <memory>
#include <optional>

#include "localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"

using mrg_modimoop_localization::wrapTo2Pi;

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization")
  {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/modimoop/localization/odometry/global", 10,
      std::bind(&LocalizationNode::odometryCallback, this, std::placeholders::_1));

    anemometer_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/modimoop/anemometer", 10,
      std::bind(&LocalizationNode::anemometerCallback, this, std::placeholders::_1));

    heading_rad_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/heading_rad", 10);

    heading_deg_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/heading_deg", 10);

    global_loc_pub_ = 
      this->create_publisher<geometry_msgs::msg::Vector3>("/modimoop/localization/global_pos", 10);

    yaw_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/yaw_rad", 10);

    twa_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/twa_rad", 10);

    tw_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3>("/modimoop/localization/tw_vec3", 10);

    // roll_pub_ =
    //    this->create_publisher<std_msgs::msg::Float64>("/modimoop/dbg/roll_rad", 10);

    // pitch_pub_ =
    //     this->create_publisher<std_msgs::msg::Float64>("/modimoop/dbg/pitch_rad", 10);

    // yaw_pub_ =
    //    this->create_publisher<std_msgs::msg::Float64>("/modimoop/dbg/yaw_rad", 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalizationNode::publishState, this));

    RCLCPP_INFO(this->get_logger(), "Localization node started");
  }

private:
    
  void anemometerCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_anemometer_ = *msg;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    latest_odometry_ = *msg;
  }

  bool computeHeading(double & heading_rad_out)
  {
    if (!latest_odometry_.has_value()) {
      return false;
    }

    // Orientation quaternion
    double qx = latest_odometry_->pose.pose.orientation.x;
    double qy = latest_odometry_->pose.pose.orientation.y;
    double qz = latest_odometry_->pose.pose.orientation.z;
    double qw = latest_odometry_->pose.pose.orientation.w;

    double normalize_term = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (normalize_term < 1e-9) return false;
    qx = qx / normalize_term;
    qy = qy / normalize_term;
    qz = qz / normalize_term;
    qw = qw / normalize_term;

    // Linear velocity
    // double vx = latest_odometry_.twist.twist.linear.x;
    // double vy = latest_odometry_.twist.twist.linear.y;
    // double vz = latest_odometry_.twist.twist.linear.z;

    // Angular velocity
    // double wx = latest_odometry_.twist.twist.angular.x;
    // double wy = latest_odometry_.twist.twist.angular.y;
    // double wz = latest_odometry_.twist.twist.angular.z;

    // Covariances
    // const auto &pose_cov = msg->pose.covariance;   // 36 elements
    // const auto &twist_cov = msg->twist.covariance; // 36 elements

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double heading = std::atan2(siny_cosp, cosy_cosp);
    double yaw = std::atan2(siny_cosp, cosy_cosp);


    // shift data into the correct frame
    auto message = std_msgs::msg::Float64();
    message.data = yaw;
    yaw_pub_->publish(message);

    heading = wrapTo2Pi(heading);

    heading_rad_out = heading;

    return true;
  }

  bool computeTW(double &twa_rad_out, geometry_msgs::msg::Vector3 &true_wind_out)
  {
    if (!latest_anemometer_.has_value() || !latest_odometry_.has_value()) return false;

    double heading_rad = 0.0;
    if (computeHeading(heading_rad) == false) return false;

    double vx = latest_odometry_->twist.twist.linear.x;
    double vy = latest_odometry_->twist.twist.linear.y;

    const auto &wind = latest_anemometer_.value();

    double rel_wind_rad = std::atan2(-wind.y, wind.x);
    rel_wind_rad = wrapTo2Pi(rel_wind_rad);
    
    double Ax = wind.x;
    double Ay = wind.y;

    // true wind = apparent wind - boat velocity
    double Wx = Ax - vx;
    double Wy = Ay - vy;

    // The TWA is defined relative to the boat's heading; thus it is in the boat's frame
    double twa = std::atan2(Wy, Wx);

    twa = wrapTo2Pi(twa);
    twa_rad_out = twa;

    true_wind_out.x = Wx;
    true_wind_out.y = Wy;
    true_wind_out.z = 0.0;

    return true;
  }

  void publishState()
  {
    if (latest_odometry_.has_value()) {
      double x = latest_odometry_->pose.pose.position.x;
      double y = latest_odometry_->pose.pose.position.y;
      double z = latest_odometry_->pose.pose.position.z;
      auto message = geometry_msgs::msg::Vector3();

      message.x = x;
      message.y = y;
      message.z = z;

      global_loc_pub_->publish(message);      
    }

    double heading_rad = 0.0;
    if (computeHeading(heading_rad)) {
      std_msgs::msg::Float64 heading_rad_msg;
      heading_rad_msg.data = heading_rad;
      heading_rad_pub_->publish(heading_rad_msg);

      std_msgs::msg::Float64 heading_deg_msg;
      heading_deg_msg.data = heading_rad * 180.0 / M_PI;
      heading_deg_pub_->publish(heading_deg_msg);
    }
    /*if (latest_imu_.has_value()) {
      const auto & imu = latest_imu_.value();

      tf2::Quaternion q(
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      std_msgs::msg::Float64 yaw_msg;
      yaw_msg.data = yaw;
      yaw_pub_->publish(yaw_msg);
    }*/
    double twa_rad = 0.0;
    geometry_msgs::msg::Vector3 true_wind_out;
    if (computeTW(twa_rad, true_wind_out)) {
      std_msgs::msg::Float64 twa_rad_msg;
      twa_rad_msg.data = twa_rad;
      twa_pub_->publish(twa_rad_msg);
      tw_pub_->publish(true_wind_out);
    }
  }

  // Parameters
  double publish_rate_hz_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr anemometer_sub_;


  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_rad_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_deg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr global_loc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr twa_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr tw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest odometry and anemometer
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<geometry_msgs::msg::Vector3> latest_anemometer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}