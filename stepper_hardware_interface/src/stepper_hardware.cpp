#include "stepper_hardware_interface/stepper_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace stepper_hardware_interface
{

hardware_interface::CallbackReturn
StepperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  port_ = info.hardware_parameters.at("port");

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);

  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Failed to open serial port");
    return CallbackReturn::ERROR;
  }

  stepper_ = std::make_unique<motion_sdk::Stepper>(*serial_);

  if (!stepper_->ping())
  {
    RCLCPP_ERROR(logger_, "Stepper ping failed");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Stepper hardware initialized");
  return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
StepperHardware::on_activate(const rclcpp_lifecycle::State &)
{
  float angle_deg = 0.0f;

  if (stepper_ && stepper_->getPosition(angle_deg))
  {
    angle_deg/=2;
    double angle_rad = angle_deg * M_PI / 180.0;

    position_state_   = angle_rad;
    position_command_ = angle_rad;

    RCLCPP_INFO(logger_, "Initial position synced: %.2f deg", angle_deg);
  }
  else
  {
    RCLCPP_WARN(logger_, "Failed to read initial stepper position");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
StepperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_state_);

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
StepperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &position_command_);

  return command_interfaces;
}


hardware_interface::return_type
StepperHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  float angle_deg = 0.0f;

  if (stepper_ && stepper_->getPosition(angle_deg))
  {
    // Continuous DEG -> RAD
    angle_deg/=2;
    position_state_ = angle_deg * M_PI / 180.0;
  }

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type
StepperHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (stepper_)
  {
    double cmd_rad = position_command_;
    double cmd_deg = cmd_rad * 180 / M_PI * 2;

    stepper_->setPosition(static_cast<float>(cmd_deg));
  }

  return hardware_interface::return_type::OK;
}



}  // namespace stepper_hardware_interface


PLUGINLIB_EXPORT_CLASS(
  stepper_hardware_interface::StepperHardware,
  hardware_interface::SystemInterface
)
