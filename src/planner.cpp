#include "path_planner.hpp"

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

namespace mrg_modimoop_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner"),
    planner_(load_params())
  {
    heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/heading_rad",
      10,
      std::bind(&PlannerNode::heading_callback, this, _1));

    twa_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/twa_rad",
      10,
      std::bind(&PlannerNode::twa_callback, this, _1));

    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/modimoop/localization/navsat",
      10,
      std::bind(&PlannerNode::navsat_callback, this, _1));

    // Recommended additional input from your higher-level mission / waypoint manager.
    goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/modimoop/global/goal_navsat",
      10,
      std::bind(&PlannerNode::goal_callback, this, _1));

    heading_target_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/modimoop/planner/heading_targ",
      10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&PlannerNode::planner_step, this));

    RCLCPP_INFO(get_logger(), "Planner node started");
  }

private:
  PlannerParams load_params()
  {
    PlannerParams p;

    declare_parameter("g_goal", p.g_goal);
    declare_parameter("g_up", p.g_up);
    declare_parameter("g_down", p.g_down);
    declare_parameter("g_hyst", p.g_hyst);
    declare_parameter("phi_up_rad", p.phi_up_rad);
    declare_parameter("phi_down_rad", p.phi_down_rad);
    declare_parameter("window_size_m", p.window_size_m);
    declare_parameter("sample_step_deg", p.sample_step_deg);

    get_parameter("g_goal", p.g_goal);
    get_parameter("g_up", p.g_up);
    get_parameter("g_down", p.g_down);
    get_parameter("g_hyst", p.g_hyst);
    get_parameter("phi_up_rad", p.phi_up_rad);
    get_parameter("phi_down_rad", p.phi_down_rad);
    get_parameter("window_size_m", p.window_size_m);
    get_parameter("sample_step_deg", p.sample_step_deg);

    return p;
  }

  void heading_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    heading_rad_ = msg->data;
  }

  void twa_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    twa_rad_ = msg->data;
  }

  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    current_position_ = GeoPoint{msg->latitude, msg->longitude};
  }

  void goal_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    goal_position_ = GeoPoint{msg->latitude, msg->longitude};
  }

  void planner_step()
  {
    if (!heading_rad_.has_value() ||
        !twa_rad_.has_value() ||
        !current_position_.has_value() ||
        !goal_position_.has_value())
    {
      return;
    }

    PlannerInput input;
    input.current_position = *current_position_;
    input.goal_position = *goal_position_;
    input.heading_rad = *heading_rad_;
    input.twa_rad = *twa_rad_;

    const double target_heading = planner_.compute_target_heading(input);

    std_msgs::msg::Float64 out;
    out.data = target_heading;
    heading_target_pub_->publish(out);
  }

  PathPlanner planner_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr twa_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<double> heading_rad_;
  std::optional<double> twa_rad_;
  std::optional<GeoPoint> current_position_;
  std::optional<GeoPoint> goal_position_;
};

}  // namespace mrg_modimoop_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrg_modimoop_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}