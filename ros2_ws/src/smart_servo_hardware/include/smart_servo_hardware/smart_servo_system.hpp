#ifndef SMART_SERVO_HARDWARE__SMART_SERVO_SYSTEM_HPP_
#define SMART_SERVO_HARDWARE__SMART_SERVO_SYSTEM_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace smart_servo_hardware
{

class SmartServoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void state_callback(const std_msgs::msg::String::SharedPtr message);
  void control_callback(const std_msgs::msg::String::SharedPtr message);
  void publish_command(const std::string & command);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;

  double servo_position_{0.0};
  double servo_velocity_{0.0};
  double door_position_{0.0};
  double door_effort_{0.0};
  double velocity_command_{0.0};
  double last_velocity_command_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool calibration_active_{false};
};

}  // namespace smart_servo_hardware

#endif  // SMART_SERVO_HARDWARE__SMART_SERVO_SYSTEM_HPP_
