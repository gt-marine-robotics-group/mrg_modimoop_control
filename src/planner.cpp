#include "path_planner.hpp"

#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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
    goal_pos_ = Vector3D{100.0, -100.0, 0.0}; // temporary goal location

    heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/heading_rad",
      10,
      std::bind(&PlannerNode::heading_callback, this, _1));

    twa_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/twa_rad",
      10,
      std::bind(&PlannerNode::twa_callback, this, _1));

    heading_target_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/modimoop/planner/heading_targ",
      10);

    location_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/modimoop/localization/global_pos",
      10,
      std::bind(&PlannerNode::location_callback, this, _1));

    goal_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/modimoop/localization/global_pos", // temporary
      10,
      std::bind(&PlannerNode::goal_callback, this, _1));

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

  void location_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    curr_pos_ = Vector3D{msg->x, msg->y, msg->z};
  }

  void goal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // goal_pos_ = Vector3D{100.0, -100.0, 0.0}; // temporary goal for
  }

  void planner_step()
  {
    if (!heading_rad_.has_value() ||
        !twa_rad_.has_value() ||
        !curr_pos_.has_value() ||
        !goal_pos_.has_value())
    {
      /*RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for planner inputs: heading=%d twa=%d current=%d goal=%d",
        heading_rad_.has_value(),
        twa_rad_.has_value(),
        curr_pos_.has_value(),
        goal_pos_.has_value());*/

      return;
    }

    PlannerInput input;
    input.current_position = *curr_pos_;
    input.goal_position = *goal_pos_;
    input.heading_rad = *heading_rad_;
    input.twa_rad = *twa_rad_;

    const double target_heading = planner_.compute_target_heading(input);
    const PlannerDebugInfo debug = planner_.compute_debug_info(input);

    std_msgs::msg::Float64 out;
    out.data = target_heading;
    heading_target_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Wind frame: heading=%.3f rad, twa_relative=%.3f rad, wind_world=%.3f rad",
      debug.heading_rad,
      debug.twa_relative_rad,
      debug.wind_world_rad);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Chosen heading=%.3f rad | potentials: goal=%.3f upwind=%.3f downwind=%.3f hyst=%.3f total=%.3f",
      debug.target_heading_rad,
      debug.goal_potential,
      debug.upwind_potential,
      debug.downwind_potential,
      debug.hysteresis_potential,
      debug.total_potential);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Published heading=%.3f rad",
      target_heading
    );
  }

  PathPlanner planner_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr twa_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr location_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<double> heading_rad_;
  std::optional<double> twa_rad_;
  // std::optional<Vector3> current_position_;
  // std::optional<Vector3> goal_position_;

  std::optional<Vector3D> curr_pos_;
  std::optional<Vector3D> goal_pos_;
};

}  // namespace mrg_modimoop_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrg_modimoop_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}