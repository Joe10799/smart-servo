#include "smart_servo_hardware/smart_servo_system.hpp"

#include <cmath>
#include <regex>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kCommandEpsilon = 1e-4;
}  // namespace

namespace smart_servo_hardware
{

hardware_interface::CallbackReturn SmartServoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2 ||
    info_.joints[0].name != "servo_joint" ||
    info_.joints[1].name != "door_joint")
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SmartServoSystem"),
      "Expected servo_joint followed by door_joint in the ros2_control description");
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("smart_servo_hardware_interface");
  command_publisher_ =
    node_->create_publisher<std_msgs::msg::String>("/smart_servo/cmd", 10);
  state_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/smart_servo/state", 10,
    std::bind(&SmartServoSystem::state_callback, this, std::placeholders::_1));
  control_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/smart_servo/control", 10,
    std::bind(&SmartServoSystem::control_callback, this, std::placeholders::_1));
  last_command_time_ = node_->now();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SmartServoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
    "servo_joint", hardware_interface::HW_IF_POSITION, &servo_position_);
  interfaces.emplace_back(
    "servo_joint", hardware_interface::HW_IF_VELOCITY, &servo_velocity_);
  interfaces.emplace_back(
    "door_joint", hardware_interface::HW_IF_POSITION, &door_position_);
  interfaces.emplace_back(
    "door_joint", hardware_interface::HW_IF_EFFORT, &door_effort_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SmartServoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
    "servo_joint", hardware_interface::HW_IF_VELOCITY, &velocity_command_);
  return interfaces;
}

hardware_interface::CallbackReturn SmartServoSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  velocity_command_ = 0.0;
  last_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SmartServoSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  velocity_command_ = 0.0;
  publish_command("S");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SmartServoSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SmartServoSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const bool changed =
    !std::isfinite(last_velocity_command_) ||
    std::abs(velocity_command_ - last_velocity_command_) > kCommandEpsilon;
  const bool refresh_due =
    std::abs(velocity_command_) > kCommandEpsilon &&
    (node_->now() - last_command_time_).seconds() >= 1.0;
  if (calibration_active_) {
    return hardware_interface::return_type::OK;
  }
  if (!changed && !refresh_due) {
    return hardware_interface::return_type::OK;
  }

  if (std::abs(velocity_command_) <= kCommandEpsilon) {
    publish_command("S");
  } else {
    const double speed_dps = std::abs(velocity_command_) / kDegreesToRadians;
    publish_command("SPD " + std::to_string(speed_dps));
    publish_command(velocity_command_ < 0.0 ? "L" : "R");
  }

  last_velocity_command_ = velocity_command_;
  last_command_time_ = node_->now();
  return hardware_interface::return_type::OK;
}

void SmartServoSystem::state_callback(
  const std_msgs::msg::String::SharedPtr message)
{
  if (message->data == "CALIBRATION_DONE") {
    calibration_active_ = false;
    last_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  if (calibration_active_ &&
    message->data.find("calibrated=1") != std::string::npos)
  {
    calibration_active_ = false;
    last_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  }

  static const std::regex state_pattern(
    R"(servo=([-0-9.]+).*door=([-0-9.]+).*speed=([-0-9.]+).*dir=(-?[0-9]+).*torque=([-0-9.]+))");
  std::smatch match;
  if (!std::regex_search(message->data, match, state_pattern)) {
    return;
  }

  servo_position_ = std::stod(match[1].str()) * kDegreesToRadians;
  door_position_ = std::stod(match[2].str()) * kDegreesToRadians;
  servo_velocity_ =
    std::stod(match[3].str()) * std::stod(match[4].str()) * kDegreesToRadians;
  door_effort_ = std::stod(match[5].str());
}

void SmartServoSystem::control_callback(
  const std_msgs::msg::String::SharedPtr message)
{
  if (message->data == "CAL") {
    calibration_active_ = true;
    velocity_command_ = 0.0;
    publish_command("CAL");
  } else if (message->data == "RESET") {
    calibration_active_ = false;
    velocity_command_ = 0.0;
    last_velocity_command_ = 0.0;
    publish_command("RESET");
  }
}

void SmartServoSystem::publish_command(const std::string & command)
{
  std_msgs::msg::String message;
  message.data = command;
  command_publisher_->publish(message);
}

}  // namespace smart_servo_hardware

PLUGINLIB_EXPORT_CLASS(
  smart_servo_hardware::SmartServoSystem,
  hardware_interface::SystemInterface)
