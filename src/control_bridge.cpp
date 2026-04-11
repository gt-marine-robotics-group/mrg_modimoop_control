#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "boat_dynamics.hpp"
#include "control_utils.hpp"
#include "sail_model.hpp"

using std::placeholders::_1;

class ControlBridge : public rclcpp::Node
{
public:
  ControlBridge()
  : Node("control_bridge"),
    params_(sailbot_control::makeDefaultBoatParams())
  {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);

    sub_yaw_ = this->create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/yaw_rad", 10,
      std::bind(&ControlBridge::onYaw, this, _1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/modimoop/imu", 10,
      std::bind(&ControlBridge::onImu, this, _1));

    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/modimoop/joint_state", 10,
      std::bind(&ControlBridge::onJointState, this, _1));

    sub_heading_ = this->create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/heading_rad", 10,
      std::bind(&ControlBridge::onHeading, this, _1));

    pub_wingsail_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/wingsail_pos", 10);
    pub_rudder_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/rudder_pos", 10);
    pub_backwing_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/backwing_pos", 10);
    pub_debug_ = this->create_publisher<std_msgs::msg::Float64>(
      "/dbg/test", 10);

    kp_heading_ = this->declare_parameter<double>("kp_heading", 3.0);
    kd_yaw_ = this->declare_parameter<double>("kd_yaw", 0.8);
    desired_heading_rad_ = this->declare_parameter<double>(
      "desired_heading_rad", M_PI / 2.0);

    rudder_limit_rad_ = this->declare_parameter<double>(
      "rudder_limit_rad", 2.0 * M_PI);
    wingsail_limit_rad_ = this->declare_parameter<double>(
      "wingsail_limit_rad", 2.0 * M_PI);
    backwing_limit_rad_ = this->declare_parameter<double>(
      "backwing_limit_rad", 2.0 * M_PI);

    wind_angle_rad_ = this->declare_parameter<double>(
      "wind_angle_rad", M_PI / 2.0);
    wind_speed_mps_ = this->declare_parameter<double>(
      "wind_speed_mps", 4.0);

    sail_trim_gain_ = this->declare_parameter<double>("sail_trim_gain", 0.5);
    nominal_sail_offset_rad_ = this->declare_parameter<double>(
      "nominal_sail_offset_rad", 0.0);
    backwing_gain_ = this->declare_parameter<double>("backwing_gain", 1.09);

    // Temporary runtime body-state estimates until a dedicated estimator/state adapter exists.
    body_u_estimate_mps_ = this->declare_parameter<double>("body_u_estimate_mps", 2.5);
    body_v_estimate_mps_ = this->declare_parameter<double>("body_v_estimate_mps", 0.0);

    // Boat params from Python model
    declareParamInPlace("boat.m", params_.m);
    declareParamInPlace("boat.Iz", params_.Iz);
    declareParamInPlace("boat.rho_w", params_.rho_w);
    declareParamInPlace("boat.rho_a", params_.rho_a);
    declareParamInPlace("boat.S_r", params_.S_r);
    declareParamInPlace("boat.l_r", params_.l_r);
    declareParamInPlace("boat.Sf", params_.Sf);
    declareParamInPlace("boat.CDx", params_.CDx);
    declareParamInPlace("boat.Yv_coeff", params_.Yv_coeff);
    declareParamInPlace("boat.C_b", params_.C_b);
    declareParamInPlace("boat.Nv_coeff", params_.Nv_coeff);
    declareParamInPlace("boat.Va", params_.Va);
    declareParamInPlace("boat.betaA", params_.betaA);
    declareParamInPlace("boat.x_ce_m", params_.x_ce_m);
    declareParamInPlace("boat.y_ce_m", params_.y_ce_m);
    declareParamInPlace("boat.x_ce_t", params_.x_ce_t);
    declareParamInPlace("boat.y_ce_t", params_.y_ce_t);
    declareParamInPlace("boat.CLalpha_r", params_.CLalpha_r);

    sail_table_csv_ = this->declare_parameter<std::string>("sail_table_csv", "");
    if (!sail_table_csv_.empty()) {
      sail_model_ = std::make_unique<sailbot_control::TableSailModel>(
        sailbot_control::TableSailModel::fromCsv(sail_table_csv_));
      have_sail_model_ = true;
      RCLCPP_INFO(get_logger(), "Loaded sail table CSV: %s", sail_table_csv_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No sail_table_csv provided. Runtime model diagnostics will be disabled.");
    }

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControlBridge::step, this));

    RCLCPP_INFO(get_logger(), "control_bridge running at %.1f Hz", rate_hz_);
  }

private:
  template<typename T>
  void declareParamInPlace(const std::string & name, T & value)
  {
    value = this->declare_parameter<T>(name, value);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_ = *msg;
    have_imu_ = true;
  }

  void onHeading(const std_msgs::msg::Float64::SharedPtr msg)
  {
    heading_rad_ = msg->data;
    have_heading_ = true;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_ = *msg;
    have_joint_ = true;
  }

  void onYaw(const std_msgs::msg::Float64::SharedPtr msg)
  {
    yaw_rad_ = msg->data;
    have_yaw_ = true;
  }

  double getJointPos(const std::string & name, bool * ok) const
  {
    for (std::size_t i = 0; i < joint_.name.size(); ++i) {
      if (joint_.name[i] == name && i < joint_.position.size()) {
        *ok = true;
        return joint_.position[i];
      }
    }
    *ok = false;
    return 0.0;
  }

  double computeWingsailTrimHeuristic() const
  {
    const double cmd =
      sail_trim_gain_ * sailbot_control::wrapToPi(wind_angle_rad_) +
      nominal_sail_offset_rad_;
    return sailbot_control::clamp(
      cmd, -wingsail_limit_rad_, wingsail_limit_rad_);
  }

  void step()
  {
    if (!(have_imu_ && have_heading_ && have_joint_)) {
      return;
    }

    bool ok_ws = false;
    bool ok_r = false;
    bool ok_bw = false;

    const double ws_meas = getJointPos("wingsail_joint", &ok_ws);
    const double rd_meas = getJointPos("rudder_joint", &ok_r);
    const double bw_meas = getJointPos("backwing_joint", &ok_bw);

    if (!(ok_ws && ok_r && ok_bw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Missing one or more control joints in /modimoop/joint_state");
      return;
    }

    (void)bw_meas;
    (void)wind_speed_mps_;

    const double yaw_rate = imu_.angular_velocity.z;
    const double heading_error =
      sailbot_control::wrapToPi(desired_heading_rad_ - heading_rad_);

    double rudder_cmd = kp_heading_ * heading_error - kd_yaw_ * yaw_rate;
    rudder_cmd = sailbot_control::clamp(
      rudder_cmd, -rudder_limit_rad_, rudder_limit_rad_);

    const double wingsail_cmd = computeWingsailTrimHeuristic();

    double backwing_cmd = -backwing_gain_ * wingsail_cmd;
    backwing_cmd = sailbot_control::clamp(
      backwing_cmd, -backwing_limit_rad_, backwing_limit_rad_);

    if (have_sail_model_) {
      sailbot_control::BodyState x;
      x.u = body_u_estimate_mps_;
      x.v = body_v_estimate_mps_;
      x.r = yaw_rate;

      // Use commanded controls for model-side diagnostics.
      sailbot_control::ControlState u_cmd;
      u_cmd.delta_r = rudder_cmd;
      u_cmd.delta_w = wingsail_cmd;

      const auto breakdown = sailbot_control::computeForceBreakdown(
        x, u_cmd, params_, *sail_model_);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "cmd: ws=%.3f rd=%.3f bw=%.3f | model: alpha_m=%.2fdeg alpha_t=%.2fdeg gamma=%.2fdeg X=%.3f Y=%.3f N=%.3f",
        wingsail_cmd,
        rudder_cmd,
        backwing_cmd,
        breakdown.alpha_m_deg,
        breakdown.alpha_t_deg,
        sailbot_control::rad2deg(breakdown.gamma),
        breakdown.total.X,
        breakdown.total.Y,
        breakdown.total.N);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "desired=%.3f heading=%.3f err=%.3f yaw_rate=%.3f rudder=%.3f ws_meas=%.3f rd_meas=%.3f",
        desired_heading_rad_,
        heading_rad_,
        heading_error,
        yaw_rate,
        rudder_cmd,
        ws_meas,
        rd_meas);
    }

    std_msgs::msg::Float64 msg;

    msg.data = wingsail_cmd;
    pub_wingsail_->publish(msg);

    msg.data = rudder_cmd;
    pub_rudder_->publish(msg);

    msg.data = backwing_cmd;
    pub_backwing_->publish(msg);

    msg.data = 1.0;
    pub_debug_->publish(msg);
  }

  double rate_hz_{0.0};

  double desired_heading_rad_{0.0};
  double kp_heading_{0.0};
  double kd_yaw_{0.0};

  double rudder_limit_rad_{0.0};
  double wingsail_limit_rad_{0.0};
  double backwing_limit_rad_{0.0};

  double wind_angle_rad_{0.0};
  double wind_speed_mps_{0.0};

  double sail_trim_gain_{0.0};
  double nominal_sail_offset_rad_{0.0};
  double backwing_gain_{0.0};

  double body_u_estimate_mps_{0.0};
  double body_v_estimate_mps_{0.0};

  std::string sail_table_csv_;
  sailbot_control::BoatParams params_;
  std::unique_ptr<sailbot_control::TableSailModel> sail_model_;
  bool have_sail_model_{false};

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_heading_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_yaw_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wingsail_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rudder_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_backwing_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_;

  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::JointState joint_;

  double yaw_rad_{0.0};
  double heading_rad_{0.0};

  bool have_imu_{false};
  bool have_heading_{false};
  bool have_joint_{false};
  bool have_yaw_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlBridge>());
  rclcpp::shutdown();
  return 0;
}