#include "arm_bldc_controller/arm_bldc_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <limits.h>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <fcntl.h>
namespace arm_bldc_controller
{

  std::string resolveToSerial(const std::string& path)
  {
      char resolved[PATH_MAX];

      if (!realpath(path.c_str(), resolved)) {
          throw std::runtime_error("Cannot resolve device path: " + path);
      }

      std::string real_dev = resolved;

      while (true)
      {
          struct stat st;
          if (stat(real_dev.c_str(), &st) == 0 &&
              S_ISCHR(st.st_mode) &&
              access(real_dev.c_str(), R_OK | W_OK) == 0)
          {
              int fd = open(real_dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
              if (fd >= 0)
              {
                  close(fd);
                  std::cout << "[PortResolver] Device ready: "
                            << real_dev << std::endl;
                  return real_dev;
              }
          }

          // Small blocking delay
          usleep(200000); // 200ms
      }
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

  device_= resolveToSerial(device_);

  size_t joints = info.joints.size();

  pos_.resize(joints, 0.0);
  vel_.resize(joints, 0.0);
  eff_.resize(joints, 0.0);

  cmd_pos_.resize(joints, 0.0);
  cmd_vel_.resize(joints, 0.0);
  cmd_eff_.resize(joints, 0.0);

  offset_={1.24235725402832,175.2515106201172, 160.47816467285156};

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
    // if(node_ids_[i]==2){
    //   offset_[node_ids_[i]] +=p;
    // }
    pos_[i] = (p-offset_[node_ids_[i]]) * DEG2RAD;
    vel_[i] = v * DEG2RAD;
    eff_[i] = t;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmBLDCSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < actuators_.size(); ++i) {
    actuators_[i]->setDeviceToActive();

    if (node_ids_[i] == 2) {
      // Move +10 degrees from current position
      cmd_pos_[i] = pos_[i] + (30.0 * DEG2RAD);

      // Optional: set a safe velocity
      cmd_vel_[i] = 20.0 * DEG2RAD;  // 20 deg/sec
    }
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

    pos_[i] = (p-offset_[node_ids_[i]]) * DEG2RAD;
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
      static_cast<float>((cmd_pos_[i] * RAD2DEG)+ offset_[node_ids_[i]]),
      static_cast<float>(cmd_vel_[i] * RAD2DEG)
    );
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_bldc_controller

PLUGINLIB_EXPORT_CLASS(
  arm_bldc_controller::ArmBLDCSystem,
  hardware_interface::SystemInterface)
