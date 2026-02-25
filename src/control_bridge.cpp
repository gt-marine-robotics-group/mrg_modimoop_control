#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

class ControlBridge : public rclcpp::Node {
public:
  ControlBridge() : Node("control_bridge") {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/modimoop/imu", 10, std::bind(&ControlBridge::onImu, this, _1));

    sub_navsat_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/modimoop/navsat", 10, std::bind(&ControlBridge::onNavSat, this, _1));

    sub_wind_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/modimoop/anemometer", 10, std::bind(&ControlBridge::onWind, this, _1));

    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/modimoop/joint_state", 10, std::bind(&ControlBridge::onJointState, this, _1));

    pub_wingsail_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/wingsail_angle", 10);

    pub_rudder_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/rudder_angle", 10);

    pub_backwing_ = this->create_publisher<std_msgs::msg::Float64>(
      "/modimoop/cmd/backwing_angle", 10);

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControlBridge::step, this));

    RCLCPP_INFO(this->get_logger(), "control_bridge running at %.1f Hz", rate_hz_);
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) { imu_ = *msg; have_imu_ = true; }
  void onNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { navsat_ = *msg; have_navsat_ = true; }
  void onWind(const geometry_msgs::msg::Vector3::SharedPtr msg) { wind_ = *msg; have_wind_ = true; }
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) { joint_ = *msg; have_joint_ = true; }

  double getJointPos(const std::string &name, bool *ok) const {
    for (size_t i = 0; i < joint_.name.size(); i++) {
      if (joint_.name[i] == name && i < joint_.position.size()) {
        *ok = true;
        return joint_.position[i];
      }
    }
    *ok = false;
    return 0.0;
  }

  void step() {
    if (!(have_imu_ && have_wind_ && have_joint_)) return;

    bool ok_ws=false, ok_r=false, ok_bw=false;
    const double ws = getJointPos("wingsail_joint", &ok_ws);
    const double rd = getJointPos("rudder_joint", &ok_r);
    const double bw = getJointPos("backwing_joint", &ok_bw);

    if (!(ok_ws && ok_r && ok_bw)) return;

    std_msgs::msg::Float64 cmd;

    cmd.data = ws; pub_wingsail_->publish(cmd);
    cmd.data = rd; pub_rudder_->publish(cmd);
    cmd.data = bw; pub_backwing_->publish(cmd);
  }

  double rate_hz_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsat_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_wind_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wingsail_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rudder_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_backwing_;

  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::NavSatFix navsat_;
  geometry_msgs::msg::Vector3 wind_;
  sensor_msgs::msg::JointState joint_;

  bool have_imu_{false}, have_navsat_{false}, have_wind_{false}, have_joint_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlBridge>());
  rclcpp::shutdown();
  return 0;
}