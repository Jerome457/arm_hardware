#include "arm_servo_controller/arm_servo_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

float angle_to_distance(float angle_deg)
{
    // Clamp angle to valid range
    if (angle_deg < 0.0f)
        angle_deg = 0.0f;
    if (angle_deg > 220.0f)
        angle_deg = 220.0f;

    // Linear mapping
    return (angle_deg / 220.0f) * (-0.04f);
}

float distance_to_angle(float distance)
{
    // Clamp distance to valid range
    if (distance > 0.0f)
        distance = 0.0f;
    if (distance < -0.04f)
        distance = -0.04f;

    // Linear mapping
    return (distance / -0.04f) * 220.0f;
}

namespace arm_servo_controller
{

hardware_interface::CallbackReturn
ArmServoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  num_joints_ = info_.joints.size();
  gripper_id_=stoi(info_.hardware_parameters.at("gripper_id"));

  position_state_.resize(num_joints_, 0.0);
  velocity_state_.resize(num_joints_, 0.0);
  effort_state_.resize(num_joints_, 0.0);
  position_command_.resize(num_joints_, 0.0);

  port_ = info_.hardware_parameters.at("port");

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);
  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Serial open failed");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    int id = std::stoi(info_.joints[i].parameters.at("id"));
    servo_ids_.push_back(id);

    auto servo = std::make_unique<motion_sdk::Servo>(*serial_, id);
    if (!servo->ping())
    {
      RCLCPP_ERROR(logger_, "Servo ping failed id=%d", id);
      return CallbackReturn::ERROR;
    }
    // RCLCPP_INFO(logger_, "FAKE INIT for testing");

    servos_.push_back(std::move(servo));
  }

  RCLCPP_INFO(logger_, "Arm servo hardware initialized (%ld joints)", num_joints_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmServoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    si.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocity_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_EFFORT, &effort_state_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
ArmServoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    ci.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_command_[i]);
  }
  return ci;
}

hardware_interface::return_type
ArmServoHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    float deg{}, eff{}, spd{};
    servos_[i]->getPosition(deg);
    servos_[i]->getEffort(eff);
    servos_[i]->getSpeed(spd);

    if(servo_ids_[i]==gripper_id_){
      position_state_[i]=angle_to_distance(deg);
    }else{
      position_state_[i] = (deg)* M_PI / 180.0;
    }
    velocity_state_[i] = spd;
    effort_state_[i] = eff;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmServoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    // position_command_[i] = std::clamp(position_command_[i], 0.0, 300);
    if(servo_ids_[i]==gripper_id_){
      servos_[i]->setPosition(distance_to_angle(position_command_[i]));
    }else{
      servos_[i]->setPosition((position_command_[i])*180/M_PI);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_servo_controller

PLUGINLIB_EXPORT_CLASS(
  arm_servo_controller::ArmServoHardware,
  hardware_interface::SystemInterface
)
