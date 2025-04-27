#include "mock_uv_hardware/mock_uv_system.hpp"

namespace mock_uv_hardware
{

hardware_interface::CallbackReturn MockUVSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  cfg_.uv_lamp_name = info_.hardware_parameters["uv_lamp_name"];
  cfg_.potential_power = std::stod(info_.hardware_parameters["potential_power"]);

  uv_state_.cmd = 0.0;
  uv_state_.state = 0.0;
  uv_state_.sent_power = 0.0;

  RCLCPP_INFO(logger_, "MockUVSystem: Initialized with UV lamp='%s', potential power=%.2fW",
              cfg_.uv_lamp_name.c_str(), cfg_.potential_power);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockUVSystem::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "MockUVSystem: Configured.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockUVSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "MockUVSystem: Cleanup.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockUVSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "MockUVSystem: Activated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockUVSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "MockUVSystem: Deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockUVSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  uv_state_.state = uv_state_.cmd;  // simulate that lamp follows command
  if (uv_state_.state > 0.5) {
    uv_state_.sent_power = cfg_.potential_power;
  } else {
    uv_state_.sent_power = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockUVSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MockUVSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(cfg_.uv_lamp_name, HW_IF_POWER, &uv_state_.sent_power);
  state_interfaces.emplace_back(cfg_.uv_lamp_name, HW_IF_STATE, &uv_state_.state);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MockUVSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(cfg_.uv_lamp_name, HW_IF_STATE, &uv_state_.cmd);
  return command_interfaces;
}

}  // namespace mock_uv_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mock_uv_hardware::MockUVSystem, hardware_interface::SystemInterface)
