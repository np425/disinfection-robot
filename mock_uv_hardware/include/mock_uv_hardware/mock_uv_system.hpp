#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mock_uv_hardware
{

constexpr char HW_IF_POWER[] = "power";
constexpr char HW_IF_STATE[] = "state";

struct Config
{
  std::string uv_lamp_name = "";
  double potential_power = 0.0;
};

struct UVState
{
  double cmd = 0.0;
  double sent_power = 0.0;   // Radiant power (fake constant)
  double state = 0.0;   // Lamp ON/OFF
};

class MockUVSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockUVSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  Config cfg_;
  UVState uv_state_;
  rclcpp::Logger logger_ = rclcpp::get_logger("MockUVSystem");
};

}  // namespace mock_uv_hardware
