// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EDDIEBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_
#define EDDIEBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_

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

#include "eddiebot_description/arduino_comms.hpp"

namespace eddiebot_description
{
struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
};

struct Wheel {
public:
  double cmd = 0;
  double pos = 0;
  double vel = 0;
};

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // Parameters for the DiffBot simulation
  ArduinoComms comms_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  Config cfg_;
};

}  // namespace eddiebot_description

#endif  // EDDIEBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_
