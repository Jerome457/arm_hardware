#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/macros.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "nmotion_transport/actuator.hpp"
#include "nmotion_transport/interface.hpp"
#include "nmotion_transport/usb_interface.hpp"

#include <rclcpp/time.hpp>
#include <vector>
#include <string>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

namespace arm_bldc_controller
{

class ArmBLDCSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmBLDCSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  Interface *iface_{nullptr};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_pub_;

  std::vector<Actuator*> actuators_;
  std::vector<uint32_t> node_ids_;

  std::vector<double> pos_, vel_, eff_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;

  std::string device_{"/dev/ttyACM0"};
};

}  // namespace arm_bldc_controller
