#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class TempGoalPublisher : public rclcpp::Node
{
public:
  TempGoalPublisher()
  : Node("temp_goal_publisher")
  {
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 2.0);
    east_offset_m_ = this->declare_parameter<double>("east_offset_m", 100.0);
    double north_offset_m_ = this->declare_parameter<double>("north_offset_m", 0.0);

    navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/modimoop/localization/navsat",
      10,
      std::bind(&TempGoalPublisher::navsatCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/modimoop/global/goal_navsat",
      10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TempGoalPublisher::publishGoal, this));

    RCLCPP_INFO(this->get_logger(), "Temp goal publisher started");
  }

private:
  void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_navsat_ = *msg;
  }

  void publishGoal()
  {
    if (!latest_navsat_.has_value()) {
      return;
    }

    const auto & cur = latest_navsat_.value();

    sensor_msgs::msg::NavSatFix goal = cur;

    // X = East, Y = North
    const double lat_rad = cur.latitude * M_PI / 180.0;
    const double earth_radius_m = 6371000.0;

    const double dlat_rad = north_offset_m_ / earth_radius_m;
    const double dlat_deg = dlat_rad * 180.0 / M_PI;

    const double dlon_rad = east_offset_m_ / (earth_radius_m * std::cos(lat_rad));
    const double dlon_deg = dlon_rad * 180.0 / M_PI;

    goal.longitude = cur.longitude + dlon_deg;
    goal.latitude = cur.latitude + dlat_deg;
    goal.header.stamp = this->now();
    goal.header.frame_id = "world";

    goal_pub_->publish(goal);
  }

  double publish_rate_hz_;
  double east_offset_m_;
  double north_offset_m_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<sensor_msgs::msg::NavSatFix> latest_navsat_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TempGoalPublisher>());
  rclcpp::shutdown();
  return 0;
}