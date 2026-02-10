#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motion_sdk/usb_serial.hpp"
#include "motion_sdk/stepper.hpp"

namespace stepper_hardware_interface
{

class StepperHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  double position_state_{0.0};
  double position_command_{0.0};

  std::string port_;

  std::unique_ptr<motion_sdk::USBSerial> serial_;
  std::unique_ptr<motion_sdk::Stepper> stepper_;

  rclcpp::Logger logger_{rclcpp::get_logger("stepper_hardware")};
};

}  // namespace stepper_hardware_interface
