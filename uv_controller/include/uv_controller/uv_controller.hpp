#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "uv_msgs/msg/uv_state_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "uv_msgs/msg/uv_state_stamped.hpp"

#include <optional>

namespace uv_controller
{

constexpr char HW_IF_POWER[] = "power";
constexpr char HW_IF_STATE[] = "state";

struct Config
{
  std::string uv_lamp_name;
  double publish_rate;
};

class UVController : public controller_interface::ControllerInterface
{
public:
  UVController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
struct UVLampHandle
{
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> power;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> state;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> state_cmd;
};

    Config cfg_;

    std::optional<UVLampHandle> lamp_handle_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uv_cmd_sub_;
    rclcpp::Publisher<uv_msgs::msg::UVStateStamped>::SharedPtr uv_state_pub_;
    bool requested_uv_on_;
};

}  // namespace uv_controller
