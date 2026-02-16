#include "arm_bldc_controller/arm_bldc_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <limits.h>
#include <unistd.h>
#include <iostream>

namespace arm_bldc_controller
{

std::string resolveToACM(const std::string& path)
{
    char resolved[PATH_MAX];

    // Resolve symlink -> real device
    if (realpath(path.c_str(), resolved))
    {
        std::string real_dev = resolved;

        // Safety: check it's actually ACM
        if (real_dev.find("ttyACM") != std::string::npos)
        {
            std::cout << "[PortResolver] " << path
                      << " -> " << real_dev << std::endl;
            return real_dev;
        }
        else
        {
            std::cerr << "[PortResolver] WARNING: Not ACM: "
                      << real_dev << std::endl;
            return real_dev; // still return
        }
    }

    // If resolving failed, use original
    std::cerr << "[PortResolver] WARNING: Could not resolve "
              << path << ", using as-is" << std::endl;

    return path;
}

hardware_interface::CallbackReturn
ArmBLDCSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("device")) {
    device_ = info.hardware_parameters.at("device");
  }

  device_= resolveToACM(device_);

  size_t joints = info.joints.size();

  pos_.resize(joints, 0.0);
  vel_.resize(joints, 0.0);
  eff_.resize(joints, 0.0);

  cmd_pos_.resize(joints, 0.0);
  cmd_vel_.resize(joints, 0.0);
  cmd_eff_.resize(joints, 0.0);

  iface_ = new USBInterface();
  iface_->initInterface(device_);

  for (const auto & joint : info.joints) {
    uint32_t node_id =
      std::stoi(joint.parameters.at("node_id"));
    node_ids_.push_back(node_id);
    actuators_.push_back(new Actuator(node_id, iface_));
  }
  
  node_ = std::make_shared<rclcpp::Node>("arm_bldc_telemetry");
  current_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_currents", 10);

  for (size_t i = 0; i < actuators_.size(); ++i) {
    float p, v, t;
    actuators_[i]->getOutputPosition(&p);
    actuators_[i]->getOutputVelocity(&v);
    actuators_[i]->getOutputTorque(&t);

    pos_[i] = p * DEG2RAD;
    vel_[i] = v * DEG2RAD;
    eff_[i] = t;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmBLDCSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto *act : actuators_) {
    // act->clearActuatorErrors();
    act->setDeviceToActive();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmBLDCSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto *act : actuators_) {
    act->setDeviceToIdle();
  }

  if (iface_) {
    iface_->closeInterface();
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmBLDCSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < pos_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, "position", &pos_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &vel_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "effort", &eff_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmBLDCSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < cmd_pos_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, "position", &cmd_pos_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "velocity", &cmd_vel_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, "effort", &cmd_eff_[i]);
  }
  return interfaces;
}

hardware_interface::return_type
ArmBLDCSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto message = std_msgs::msg::Float32MultiArray();
  for (size_t i = 0; i < actuators_.size(); ++i) {
    float p, v, t,id, iq,temperature;
    actuators_[i]->getOutputPosition(&p);
    actuators_[i]->getOutputVelocity(&v);
    actuators_[i]->getOutputTorque(&t);
    actuators_[i]->getIdqCurrents(&id, &iq);
    actuators_[i]->getDriverTemperature(&temperature);

    uint32_t errorCode;
    actuators_[i]->getErrorCode(&errorCode);

    if(0<errorCode && errorCode<=256) RCLCPP_WARN(rclcpp::get_logger("ArmBLDCSystem"), "Motor no:%i CODE :%i", i, errorCode);

    pos_[i] = p * DEG2RAD;
    vel_[i] = v * DEG2RAD;
    eff_[i] = t;
    message.data.push_back(id);
    message.data.push_back(iq);
    message.data.push_back(temperature);
  }
  current_pub_->publish(message);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmBLDCSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < actuators_.size(); ++i) {
    actuators_[i]->setPositionControl(
      static_cast<float>(cmd_pos_[i] * RAD2DEG),
      40.0f
    );
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_bldc_controller

PLUGINLIB_EXPORT_CLASS(
  arm_bldc_controller::ArmBLDCSystem,
  hardware_interface::SystemInterface)
