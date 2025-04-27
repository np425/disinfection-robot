#include "disinfectbot_hardware/disinfectbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace disinfectbot_hardware
{
hardware_interface::CallbackReturn DisinfectBotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.uv_lamp_name = info_.hardware_parameters["uv_lamp_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DisinfectBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.uv_lamp_name, disinfectbot_hardware::HW_IF_POWER, &uv_state_.power));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.uv_lamp_name, disinfectbot_hardware::HW_IF_STATE, &uv_state_.state));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DisinfectBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.uv_lamp_name, disinfectbot_hardware::HW_IF_STATE, &uv_state_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DisinfectBotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DisinfectBotSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DisinfectBotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DisinfectBotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DisinfectBotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l_.pos, wheel_r_.pos, wheel_l_.vel, wheel_r_.vel);

  bool state;
  comms_.read_uv_state(state, uv_state_.power);
  uv_state_.state = state;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DisinfectBotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.set_motor_velocities(wheel_l_.cmd, wheel_r_.cmd);
  RCLCPP_INFO(logger_, "Set motor velocities: left = %f, right = %f", wheel_l_.cmd, wheel_r_.cmd);

  comms_.set_uv_state(uv_state_.cmd > 0.5);

  return hardware_interface::return_type::OK;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  disinfectbot_hardware::DisinfectBotSystem, hardware_interface::SystemInterface)
