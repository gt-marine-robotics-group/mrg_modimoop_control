#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double x, double lo, double hi)
{
  return std::min(std::max(x, lo), hi);
}

double wrap_to_pi(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double wrap_to_2pi(double angle)
{
  double out = std::fmod(angle, 2.0 * kPi);
  if (out < 0.0) {
    out += 2.0 * kPi;
  }
  return out;
}

double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

bool parse_bool_like(const std::string & s)
{
  return s == "True" || s == "true" || s == "1" || s == "TRUE";
}

std::vector<std::string> parse_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string current;
  bool in_quotes = false;

  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
        current.push_back('"');
        ++i;
      } else {
        in_quotes = !in_quotes;
      }
    } else if (c == ',' && !in_quotes) {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }

  fields.push_back(current);
  return fields;
}

double get_csv_double(
  const std::unordered_map<std::string, std::size_t> & header,
  const std::vector<std::string> & fields,
  const std::string & name)
{
  const auto it = header.find(name);
  if (it == header.end() || it->second >= fields.size()) {
    throw std::runtime_error("Missing CSV column: " + name);
  }
  return std::stod(fields[it->second]);
}

std::string get_csv_string(
  const std::unordered_map<std::string, std::size_t> & header,
  const std::vector<std::string> & fields,
  const std::string & name)
{
  const auto it = header.find(name);
  if (it == header.end() || it->second >= fields.size()) {
    return "";
  }
  return fields[it->second];
}
}  // namespace

class LqrHeadingController : public rclcpp::Node
{
public:
  LqrHeadingController()
  : Node("lqr_heading_controller")
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/modimoop/localization/odometry/global");
    heading_topic_ = declare_parameter<std::string>(
      "heading_topic", "/modimoop/localization/heading_rad");
    target_heading_topic_ = declare_parameter<std::string>(
      "target_heading_topic", "/modimoop/planner/heading_targ");
    wind_body_topic_ = declare_parameter<std::string>(
      "wind_body_topic", "/modimoop/localization/tw_vec3");
    rudder_command_topic_ = declare_parameter<std::string>(
      "rudder_command_topic", "/modimoop/cmd/rudder_pos");
    backwing_command_topic_ = declare_parameter<std::string>(
      "backwing_command_topic", "/modimoop/cmd/backwing_pos");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/modimoop/control/lqr_heading/debug");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    sensor_timeout_s_ = declare_parameter<double>("sensor_timeout_s", 0.35);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 2.0);
    wind_timeout_s_ = declare_parameter<double>("wind_timeout_s", 1.0);

    heading_kp_ = declare_parameter<double>("heading_kp", 0.8);
    heading_kd_ = declare_parameter<double>("heading_kd", 0.15);
    r_cmd_max_ = declare_parameter<double>("r_cmd_max_rad_s", deg2rad(20.0));

    delta_r_max_ = declare_parameter<double>("delta_r_max_rad", deg2rad(20.0));
    delta_w_max_ = declare_parameter<double>("delta_w_max_rad", deg2rad(12.0));
    rudder_sign_ = declare_parameter<double>("rudder_sign", 1.0);
    backwing_sign_ = declare_parameter<double>("backwing_sign", 1.0);

    safe_rudder_rad_ = declare_parameter<double>("safe_rudder_rad", 0.0);
    safe_backwing_rad_ = declare_parameter<double>("safe_backwing_rad", 0.0);
    publish_safe_on_stale_ = declare_parameter<bool>("publish_safe_on_stale", true);

    // hard fail on bad load
    require_schedule_load_ = declare_parameter<bool>("require_schedule_load", true);

    use_gain_schedule_ = declare_parameter<bool>("use_gain_schedule", false);
    schedule_csv_path_ = declare_parameter<std::string>("schedule_csv_path", "");
    schedule_trim_speed_ = declare_parameter<double>("schedule_trim_speed", 3.0);
    schedule_trim_tolerance_ = declare_parameter<double>("schedule_trim_tolerance", 1e-6);
    schedule_speed_scale_mps_ = declare_parameter<double>("schedule_speed_scale_mps", 1.0);
    schedule_angle_scale_rad_ = declare_parameter<double>("schedule_angle_scale_rad", deg2rad(30.0));
    require_wind_for_schedule_ = declare_parameter<bool>("require_wind_for_schedule", true);

    const std::vector<double> default_x_trim = {3.0, 0.0, 0.0};
    const std::vector<double> default_u_trim = {0.0, 0.0};
    const std::vector<double> default_k = {
      0.54565928, 5.17673327, 10.41498649,
      1.24463216, 0.21089667, -0.94761705};

    x_trim_ = get_fixed_vector("x_trim", default_x_trim, 3);
    u_trim_ = get_fixed_vector("u_trim", default_u_trim, 2);
    k_row_major_ = get_fixed_vector("K_row_major", default_k, 6);

    if (use_gain_schedule_) {
      load_schedule_from_csv(schedule_csv_path_);
      if (!schedule_points_.empty()) {
        apply_schedule_point(schedule_points_.front());
        RCLCPP_INFO(
          get_logger(),
          "Initialized active gains from first schedule row: Va=%.3f m/s, beta=%.3f rad, trim=%.3f m/s",
          active_schedule_wind_speed_, active_schedule_wind_angle_rad_, active_schedule_trim_speed_);
      }
    }

    rudder_pub_ = create_publisher<std_msgs::msg::Float64>(rudder_command_topic_, 10);
    backwing_pub_ = create_publisher<std_msgs::msg::Float64>(backwing_command_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(), std::bind(&LqrHeadingController::odom_cb, this, _1));
    heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      heading_topic_, 10, std::bind(&LqrHeadingController::heading_cb, this, _1));
    target_heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      target_heading_topic_, 10, std::bind(&LqrHeadingController::target_heading_cb, this, _1));
    wind_body_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      wind_body_topic_, rclcpp::SensorDataQoS(), std::bind(&LqrHeadingController::wind_body_cb, this, _1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));
    timer_ = create_wall_timer(period, std::bind(&LqrHeadingController::control_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "LQR heading controller started. Publishing rudder to %s and backwing to %s",
      rudder_command_topic_.c_str(),
      backwing_command_topic_.c_str());
  }

private:
  struct SchedulePoint
  {
    double apparent_wind_speed{};
    double apparent_wind_angle_rad{};
    double trim_speed{};
    std::array<double, 3> x_trim{};
    std::array<double, 2> u_trim{};
    std::array<double, 6> K{};
  };

  std::vector<double> get_fixed_vector(
    const std::string & name,
    const std::vector<double> & default_value,
    std::size_t expected_size)
  {
    const auto value = declare_parameter<std::vector<double>>(name, default_value);
    if (value.size() != expected_size) {
      RCLCPP_FATAL(
        get_logger(), "Parameter '%s' must have size %zu; got %zu",
        name.c_str(), expected_size, value.size());
      throw std::runtime_error("Invalid fixed-size vector parameter: " + name);
    }
    return value;
  }

  void load_schedule_from_csv(const std::string & path)
  {
    if (path.empty()) {
      const std::string msg =
        "use_gain_schedule is true, but schedule_csv_path is empty.";
      RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
      if (require_schedule_load_) {
        throw std::runtime_error(msg);
      }
      use_gain_schedule_ = false;
      return; 
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      const std::string msg = "Could not open schedule CSV: " + path;
      RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
      if (require_schedule_load_) {
        throw std::runtime_error(msg);
      }
      use_gain_schedule_ = false;
      return;
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
      RCLCPP_ERROR(get_logger(), "Schedule CSV is empty: %s", path.c_str());
      use_gain_schedule_ = false;
      return;
    }

    const auto header_fields = parse_csv_line(header_line);
    std::unordered_map<std::string, std::size_t> header;
    for (std::size_t i = 0; i < header_fields.size(); ++i) {
      header[header_fields[i]] = i;
    }

    std::size_t total_rows = 0;
    std::size_t skipped_rows = 0;
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      ++total_rows;

      const auto fields = parse_csv_line(line);
      try {
        if (!parse_bool_like(get_csv_string(header, fields, "valid"))) {
          ++skipped_rows;
          continue;
        }

        const double trim_speed = get_csv_double(header, fields, "trim_speed");
        if (std::abs(trim_speed - schedule_trim_speed_) > schedule_trim_tolerance_) {
          ++skipped_rows;
          continue;
        }

        SchedulePoint p;
        p.apparent_wind_speed = get_csv_double(header, fields, "apparent_wind_speed");
        p.apparent_wind_angle_rad = get_csv_double(header, fields, "apparent_wind_angle_rad");
        p.trim_speed = trim_speed;
        p.x_trim = {
          get_csv_double(header, fields, "x_trim_u"),
          get_csv_double(header, fields, "x_trim_v"),
          get_csv_double(header, fields, "x_trim_r")};
        p.u_trim = {
          get_csv_double(header, fields, "u_trim_rudder"),
          get_csv_double(header, fields, "u_trim_backwing")};
        p.K = {
          get_csv_double(header, fields, "K00"),
          get_csv_double(header, fields, "K01"),
          get_csv_double(header, fields, "K02"),
          get_csv_double(header, fields, "K10"),
          get_csv_double(header, fields, "K11"),
          get_csv_double(header, fields, "K12")};
        schedule_points_.push_back(p);
      } catch (const std::exception & exc) {
        ++skipped_rows;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping malformed schedule row: %s", exc.what());
      }
    }

    if (schedule_points_.empty()) {
      std::ostringstream oss;
      oss << "Loaded schedule CSV " << path
          << ", but found no valid rows with trim_speed "
          << schedule_trim_speed_
          << ". total_rows=" << total_rows
          << ", skipped_rows=" << skipped_rows;

      const std::string msg = oss.str();
      RCLCPP_FATAL(get_logger(), "%s", msg.c_str());

      if (require_schedule_load_) {
        throw std::runtime_error(msg);
      }

      use_gain_schedule_ = false;
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu valid schedule rows at trim_speed %.3f m/s from %s (%zu total rows, %zu skipped).",
      schedule_points_.size(), schedule_trim_speed_, path.c_str(), total_rows, skipped_rows);
  }

  void apply_schedule_point(const SchedulePoint & p)
  {
    x_trim_ = {p.x_trim[0], p.x_trim[1], p.x_trim[2]};
    u_trim_ = {p.u_trim[0], p.u_trim[1]};
    k_row_major_ = {p.K[0], p.K[1], p.K[2], p.K[3], p.K[4], p.K[5]};

    active_schedule_wind_speed_ = p.apparent_wind_speed;
    active_schedule_wind_angle_rad_ = p.apparent_wind_angle_rad;
    active_schedule_trim_speed_ = p.trim_speed;
    has_active_schedule_point_ = true;
  }

  void update_gain_schedule()
  {
    if (!use_gain_schedule_ || schedule_points_.empty()) {
      return;
    }

    if (!has_wind_body_) {
      return;
    }

    const double wind_speed = std::hypot(wind_x_b_, wind_y_b_);
    const double wind_angle = wrap_to_pi(std::atan2(wind_y_b_, wind_x_b_));

    if (!std::isfinite(wind_speed) || !std::isfinite(wind_angle)) {
      return;
    }

    const double speed_scale = std::max(schedule_speed_scale_mps_, 1e-6);
    const double angle_scale = std::max(schedule_angle_scale_rad_, 1e-6);

    const SchedulePoint * best = nullptr;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto & p : schedule_points_) {
      const double dv = (wind_speed - p.apparent_wind_speed) / speed_scale;
      const double da = wrap_to_pi(wind_angle - p.apparent_wind_angle_rad) / angle_scale;
      const double score = dv * dv + da * da;
      if (score < best_score) {
        best_score = score;
        best = &p;
      }
    }

    if (best == nullptr) {
      return;
    }

    const bool changed =
      !has_active_schedule_point_ ||
      std::abs(best->apparent_wind_speed - active_schedule_wind_speed_) > 1e-9 ||
      std::abs(wrap_to_pi(best->apparent_wind_angle_rad - active_schedule_wind_angle_rad_)) > 1e-9 ||
      std::abs(best->trim_speed - active_schedule_trim_speed_) > 1e-9;

    if (changed) {
      apply_schedule_point(*best);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Selected schedule row: current wind Va=%.3f m/s beta=%.3f rad -> table Va=%.3f beta=%.3f trim=%.3f",
        wind_speed, wind_angle, active_schedule_wind_speed_, active_schedule_wind_angle_rad_, active_schedule_trim_speed_);
    }
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Assumption: odometry twist is already expressed in the boat/hull/body frame.
    u_ = msg->twist.twist.linear.x;
    v_ = msg->twist.twist.linear.y;
    r_ = msg->twist.twist.angular.z;
    last_odom_time_ = now();
    has_odom_ = true;
  }

  void heading_cb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    psi_ = wrap_to_pi(msg->data);
    last_heading_time_ = now();
    has_heading_ = true;

    if (!has_target_heading_) {
      // Until the planner publishes a command, hold the current heading.
      psi_cmd_ = psi_;
      last_target_heading_time_ = now();
      has_target_heading_ = true;
    }
  }

  void target_heading_cb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    psi_cmd_ = wrap_to_pi(msg->data);
    last_target_heading_time_ = now();
    has_target_heading_ = true;
  }

  void wind_body_cb(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    wind_x_b_ = msg->x;
    wind_y_b_ = msg->y;
    last_wind_time_ = now();
    has_wind_body_ = true;
  }

  bool stale(const rclcpp::Time & stamp, double timeout_s) const
  {
    return (now() - stamp).seconds() > timeout_s;
  }

  void publish_safe_command()
  {
    std_msgs::msg::Float64 rudder_msg;
    std_msgs::msg::Float64 backwing_msg;
    rudder_msg.data = rudder_sign_ * safe_rudder_rad_;
    backwing_msg.data = backwing_sign_ * safe_backwing_rad_;
    rudder_pub_->publish(rudder_msg);
    backwing_pub_->publish(backwing_msg);
  }

  void control_tick()
  {
    if (!has_odom_ || !has_heading_ || !has_target_heading_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for odom, heading, and target heading before publishing closed-loop commands.");
      if (publish_safe_on_stale_) {
        publish_safe_command();
      }
      return;
    }

    if (stale(last_odom_time_, sensor_timeout_s_) || stale(last_heading_time_, sensor_timeout_s_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "State estimate is stale; publishing safe actuator command.");
      if (publish_safe_on_stale_) {
        publish_safe_command();
      }
      return;
    }

    if (use_gain_schedule_ && require_wind_for_schedule_) {
      if (!has_wind_body_ || stale(last_wind_time_, wind_timeout_s_)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Wind estimate is missing/stale and gain schedule requires wind; publishing safe actuator command.");
        if (publish_safe_on_stale_) {
          publish_safe_command();
        }
        return;
      }
    }

    if (stale(last_target_heading_time_, target_timeout_s_)) {
      // Planner command is stale: hold current heading instead of chasing an old target.
      psi_cmd_ = psi_;
      last_target_heading_time_ = now();
    }

    update_gain_schedule();

    double psi_err = wrap_to_pi(psi_cmd_ - psi_);

    // a deadband to not calculate until heading has changed more than 5 degrees
    const double heading_deadband = deg2rad(5);

    if (std::abs(psi_err) < heading_deadband) {
      psi_err = 0.0;
    }

    const double psi_err_dot = -r_;
    const double r_cmd = clamp(
      heading_kp_ * psi_err + heading_kd_ * psi_err_dot,
      -r_cmd_max_, r_cmd_max_);

    const std::array<double, 3> x_tilde = {
      u_ - x_trim_[0],
      v_ - x_trim_[1],
      r_ - r_cmd};

    // K_row_major = [K00 K01 K02 K10 K11 K12].
    double delta_r = u_trim_[0] - (
      k_row_major_[0] * x_tilde[0] +
      k_row_major_[1] * x_tilde[1] +
      k_row_major_[2] * x_tilde[2]);

    double delta_w = u_trim_[1] - (
      k_row_major_[3] * x_tilde[0] +
      k_row_major_[4] * x_tilde[1] +
      k_row_major_[5] * x_tilde[2]);

    delta_r = clamp(delta_r, -delta_r_max_, delta_r_max_);
    delta_w = clamp(delta_w, -delta_w_max_, delta_w_max_);

    const double actuator_rudder = rudder_sign_ * delta_r;
    const double actuator_backwing = backwing_sign_ * delta_w;

    std_msgs::msg::Float64 rudder_msg;
    std_msgs::msg::Float64 backwing_msg;
    rudder_msg.data = actuator_rudder;
    backwing_msg.data = actuator_backwing;
    rudder_pub_->publish(rudder_msg);
    backwing_pub_->publish(backwing_msg);

    const double wind_speed = has_wind_body_ ? std::hypot(wind_x_b_, wind_y_b_) : std::numeric_limits<double>::quiet_NaN();
    const double wind_angle = has_wind_body_ ? wrap_to_pi(std::atan2(wind_y_b_, wind_x_b_)) : std::numeric_limits<double>::quiet_NaN();

    std_msgs::msg::Float64MultiArray dbg_msg;
    dbg_msg.data = {
      psi_, psi_cmd_, psi_err, r_cmd,
      u_, v_, r_,
      delta_r, delta_w,
      has_wind_body_ ? wind_x_b_ : std::numeric_limits<double>::quiet_NaN(),
      has_wind_body_ ? wind_y_b_ : std::numeric_limits<double>::quiet_NaN(),
      wind_speed,
      wind_angle,
      active_schedule_wind_speed_,
      active_schedule_wind_angle_rad_,
      active_schedule_trim_speed_,
      use_gain_schedule_ ? 1.0 : 0.0,
      static_cast<double>(schedule_points_.size()),
      has_active_schedule_point_ ? 1.0 : 0.0,
      schedule_trim_speed_};
    debug_pub_->publish(dbg_msg);
  }

  std::string odom_topic_;
  std::string heading_topic_;
  std::string target_heading_topic_;
  std::string wind_body_topic_;
  std::string rudder_command_topic_;
  std::string backwing_command_topic_;
  std::string debug_topic_;
  std::string schedule_csv_path_;

  double control_rate_hz_{};
  double sensor_timeout_s_{};
  double target_timeout_s_{};
  double wind_timeout_s_{};
  double heading_kp_{};
  double heading_kd_{};
  double r_cmd_max_{};
  double delta_r_max_{};
  double delta_w_max_{};
  double rudder_sign_{};
  double backwing_sign_{};
  double safe_rudder_rad_{};
  double safe_backwing_rad_{};
  double schedule_trim_speed_{};
  double schedule_trim_tolerance_{};
  double schedule_speed_scale_mps_{};
  double schedule_angle_scale_rad_{};
  bool publish_safe_on_stale_{};
  bool use_gain_schedule_{};
  bool require_wind_for_schedule_{};

  std::vector<double> x_trim_;
  std::vector<double> u_trim_;
  std::vector<double> k_row_major_;
  std::vector<SchedulePoint> schedule_points_;

  double u_{};
  double v_{};
  double r_{};
  double psi_{};
  double psi_cmd_{};
  double wind_x_b_{};
  double wind_y_b_{};

  double active_schedule_wind_speed_{std::numeric_limits<double>::quiet_NaN()};
  double active_schedule_wind_angle_rad_{std::numeric_limits<double>::quiet_NaN()};
  double active_schedule_trim_speed_{std::numeric_limits<double>::quiet_NaN()};

  bool has_odom_{false};
  bool has_heading_{false};
  bool has_target_heading_{false};
  bool has_wind_body_{false};
  bool has_active_schedule_point_{false};

  // failure error
  bool require_schedule_load_{true};

  rclcpp::Time last_odom_time_{};
  rclcpp::Time last_heading_time_{};
  rclcpp::Time last_target_heading_time_{};
  rclcpp::Time last_wind_time_{};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr backwing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_heading_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_body_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrHeadingController>());
  rclcpp::shutdown();
  return 0;
}