#include <array>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <vector>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

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

    /*sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/modimoop/imu", 10,
      std::bind(&ControlBridge::onImu, this, _1));*/

    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/modimoop/joint_state", 10,
      std::bind(&ControlBridge::onJointState, this, _1));

    sub_heading_ = this->create_subscription<std_msgs::msg::Float64>(
      "/modimoop/localization/heading_rad", 10,
      std::bind(&ControlBridge::onHeading, this, _1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/modimoop/localization/odometry/global", 10,
      std::bind(&ControlBridge::onOdom, this, _1));

    pub_wingsail_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/wingsail_pos", 10);
    pub_rudder_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/rudder_pos", 10);
    pub_backwing_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/backwing_pos", 10);
    pub_debug_ = this->create_publisher<std_msgs::msg::Float64>(
      "/dbg/test", 10);

    // Python-equivalent command limits from sim_closed_loop_py.py
    rudder_limit_rad_ = this->declare_parameter<double>(
      "rudder_limit_rad", sailbot_control::deg2rad(20.0));
    wingsail_limit_rad_ = this->declare_parameter<double>(
      "wingsail_limit_rad", sailbot_control::deg2rad(12.0));

    backwing_gain_ = this->declare_parameter<double>("backwing_gain", 1.09);
    backwing_limit_rad_ = this->declare_parameter<double>(
      "backwing_limit_rad", std::abs(backwing_gain_) * wingsail_limit_rad_);

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

    x_trim_ = this->declare_parameter<std::vector<double>>(
      "controller.x_trim", std::vector<double>{2.5, 0.0, 0.0});
    u_trim_ = this->declare_parameter<std::vector<double>>(
      "controller.u_trim", std::vector<double>{0.0, 0.0});
    K_ = this->declare_parameter<std::vector<double>>(
      "controller.K", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (x_trim_.size() != 3) {
      throw std::runtime_error("controller.x_trim must have 3 entries");
    }
    if (u_trim_.size() != 2) {
      throw std::runtime_error("controller.u_trim must have 2 entries");
    }
    if (K_.size() != 6) {
      throw std::runtime_error("controller.K must have 6 entries in row-major 2x3 order");
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

  sailbot_control::ControlState computeClosedLoopCommand(
    const sailbot_control::BodyState & x) const
  {
    const double e0 = x.u - x_trim_[0];
    const double e1 = x.v - x_trim_[1];
    const double e2 = x.r - x_trim_[2];

    sailbot_control::ControlState u_cmd;
    u_cmd.delta_r = u_trim_[0] - (K_[0] * e0 + K_[1] * e1 + K_[2] * e2);
    u_cmd.delta_w = u_trim_[1] - (K_[3] * e0 + K_[4] * e1 + K_[5] * e2);

    u_cmd.delta_r = sailbot_control::clamp(
      u_cmd.delta_r, -rudder_limit_rad_, rudder_limit_rad_);
    u_cmd.delta_w = sailbot_control::clamp(
      u_cmd.delta_w, -wingsail_limit_rad_, wingsail_limit_rad_);
    return u_cmd;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
   {
     odom_ = *msg;
     have_odom_ = true;
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

  void step()
  {
    if (!(have_odom_ && have_joint_ && have_sail_model_)) {
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

    // hardcoded corrected state for now
    sailbot_control::BodyState x;
    x.u = odom_.twist.twist.linear.y;
    x.v = -odom_.twist.twist.linear.x;
    x.r = odom_.twist.twist.angular.z;

    const auto u_cmd = computeClosedLoopCommand(x);

    const double rudder_cmd = u_cmd.delta_r;
    const double wingsail_cmd = u_cmd.delta_w;
    double backwing_cmd = -backwing_gain_ * wingsail_cmd;
    backwing_cmd = sailbot_control::clamp(
      backwing_cmd, -backwing_limit_rad_, backwing_limit_rad_);

    const sailbot_control::ControlState model_u{
      rudder_cmd,
      wingsail_cmd
    };

    const auto breakdown = sailbot_control::computeForceBreakdown(
      x, model_u, params_, *sail_model_);

    /*RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "x=[%.3f %.3f %.3f] u_cmd=[%.3f %.3f] bw=%.3f | meas ws=%.3f rd=%.3f bw=%.3f | X=%.3f Y=%.3f N=%.3f",
      x.u, x.v, x.r,
      rudder_cmd, wingsail_cmd, backwing_cmd,
      ws_meas, rd_meas, bw_meas,
      breakdown.total.X, breakdown.total.Y, breakdown.total.N);*/

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

  double rudder_limit_rad_{0.0};
  double wingsail_limit_rad_{0.0};
  double backwing_limit_rad_{0.0};
  double backwing_gain_{0.0};

  std::vector<double> x_trim_;
  std::vector<double> u_trim_;
  std::vector<double> K_;

  std::string sail_table_csv_;
  sailbot_control::BoatParams params_;
  std::unique_ptr<sailbot_control::TableSailModel> sail_model_;
  bool have_sail_model_{false};

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_heading_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wingsail_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rudder_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_backwing_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_debug_;

  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_;

  double yaw_rad_{0.0};
  double heading_rad_{0.0};

  bool have_heading_{false};
  bool have_joint_{false};
  bool have_odom_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlBridge>());
  rclcpp::shutdown();
  return 0;
}