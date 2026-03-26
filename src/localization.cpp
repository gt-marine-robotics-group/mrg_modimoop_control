#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/float64.hpp"

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization")
  {
    // Parameters
    declination_rad_ = this->declare_parameter<double>("declination_rad", 0.0);
    use_tilt_compensation_ = this->declare_parameter<bool>("use_tilt_compensation", true);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/modimoop/imu", 10,
      std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/modimoop/mag", 10,
      std::bind(&LocalizationNode::magCallback, this, std::placeholders::_1));

    navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/modimoop/navsat", 10,
      std::bind(&LocalizationNode::navsatCallback, this, std::placeholders::_1));

    heading_rad_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/heading_rad", 10);

    heading_deg_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/modimoop/localization/heading_deg", 10);

    navsat_pub_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("/modimoop/localization/navsat", 10);

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
  static double wrapTo2Pi(double angle)
  {
    angle = std::fmod(angle, 2.0 * M_PI);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
  }

  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    latest_mag_ = *msg;
  }

  void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_navsat_ = *msg;
  }

  bool computeHeading(double & heading_rad_out)
  {
    if (!latest_mag_.has_value()) {
      return false;
    }

    const auto & mag = latest_mag_.value();

    double mx = mag.magnetic_field.x;
    double my = mag.magnetic_field.y;
    double mz = mag.magnetic_field.z;

    double heading = 0.0;

    if (use_tilt_compensation_ && latest_imu_.has_value()) {
      const auto & imu = latest_imu_.value();

      // Extract roll, pitch, yaw from IMU orientation
      tf2::Quaternion q(
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // Tilt compensation
      double mx_comp = mx * std::cos(pitch) + mz * std::sin(pitch);
      double my_comp =
        mx * std::sin(roll) * std::sin(pitch) +
        my * std::cos(roll) -
        mz * std::sin(roll) * std::cos(pitch);

      // need to use -y to get the orientation I want
      // lower degrees in Quad 1; higher degrees towards Quad 4 in the x-y plane
      heading = std::atan2(-my_comp, mx_comp);
    } else {
      // Flat assumption
      heading = std::atan2(-my, mx);
    }

    heading += declination_rad_;
    heading = wrapTo2Pi(heading);

    heading_rad_out = heading;
    return true;
  }

  void publishState()
  {
    if (latest_navsat_.has_value()) {
      navsat_pub_->publish(latest_navsat_.value());
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

      std_msgs::msg::Float64 roll_msg;
      roll_msg.data = roll;
      roll_pub_->publish(roll_msg);

      std_msgs::msg::Float64 pitch_msg;
      pitch_msg.data = pitch;
      pitch_pub_->publish(pitch_msg);

      std_msgs::msg::Float64 yaw_msg;
      yaw_msg.data = yaw;
      yaw_pub_->publish(yaw_msg);
    }*/
  }

  // Parameters
  double declination_rad_;
  bool use_tilt_compensation_;
  double publish_rate_hz_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_rad_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_deg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;

  // Debug Publishers
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest sensor values
  std::optional<sensor_msgs::msg::Imu> latest_imu_;
  std::optional<sensor_msgs::msg::MagneticField> latest_mag_;
  std::optional<sensor_msgs::msg::NavSatFix> latest_navsat_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}