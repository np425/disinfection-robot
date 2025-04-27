#include "uv_controller/uv_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "controller_interface/helpers.hpp"

namespace uv_controller
{

UVController::UVController()
: controller_interface::ControllerInterface()
{}

controller_interface::CallbackReturn UVController::on_init()
{
  auto_declare<std::string>("uv_lamp_name", "uv_lamp_link");
  auto_declare<double>("publish_rate", 5.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UVController::on_configure(const rclcpp_lifecycle::State &)
{
  cfg_.uv_lamp_name = get_node()->get_parameter("uv_lamp_name").as_string();
  cfg_.publish_rate = get_node()->get_parameter("publish_rate").as_double();

  requested_uv_on_ = false;
  uv_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "cmd_uv", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      requested_uv_on_ = msg->data;
      RCLCPP_DEBUG(get_node()->get_logger(), "UVController: Received cmd_uv = %s", requested_uv_on_ ? "true" : "false");
    });

  uv_state_pub_ = get_node()->create_publisher<uv_msgs::msg::UVStateStamped>("/uv_state", rclcpp::QoS(10));

  RCLCPP_INFO(get_node()->get_logger(), "UVController: on_configure completed. uv_lamp_name='%s', publish_rate=%.2f Hz",
              cfg_.uv_lamp_name.c_str(), cfg_.publish_rate);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration UVController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_STATE);
  return config;
}

controller_interface::InterfaceConfiguration UVController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_STATE);
  config.names.push_back(cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_POWER);
  return config;
}

controller_interface::CallbackReturn UVController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto power_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&](const auto & interface) {
      return interface.get_name() == cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_POWER;
    });
  
  const auto state_it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&](const auto & interface) {
      return interface.get_name() == cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_STATE;
    });
  
  const auto cmd_it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const auto & interface) {
      return interface.get_name() == cfg_.uv_lamp_name + "/" + uv_controller::HW_IF_STATE;
    });
  
  if (power_it == state_interfaces_.end() ||
      state_it == state_interfaces_.end() ||
      cmd_it == command_interfaces_.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "UVController: Failed to find required hardware interfaces during activation.");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  lamp_handle_.emplace(UVLampHandle{
    std::cref(*power_it),
    std::cref(*state_it),
    std::ref(*cmd_it)
  });

  RCLCPP_INFO(get_node()->get_logger(), "UVController: Successfully activated hardware interfaces.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UVController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UVController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!lamp_handle_) {
    RCLCPP_ERROR(get_node()->get_logger(), "UVController: update called without valid lamp handle.");
    return controller_interface::return_type::ERROR;
  }

  static bool first_update = true;
  static rclcpp::Time last_publish_time;

  if (first_update) {
    last_publish_time = time;
    first_update = false;
  }

  lamp_handle_->state_cmd.get().set_value(requested_uv_on_ ? 1.0 : 0.0);

  double publish_period = 1.0 / cfg_.publish_rate;
  if ((time - last_publish_time).seconds() >= publish_period) {
    if (uv_state_pub_->get_subscription_count() > 0) {
      uv_msgs::msg::UVStateStamped msg;
      msg.header.stamp = time;
      msg.header.frame_id = cfg_.uv_lamp_name;
      msg.is_on = lamp_handle_->state.get().get_value();
      msg.power_radiant = lamp_handle_->power.get().get_value();
      uv_state_pub_->publish(msg);
    }
    last_publish_time = time;
  }

  return controller_interface::return_type::OK;
}

}  // namespace uv_controller

PLUGINLIB_EXPORT_CLASS(
  uv_controller::UVController, controller_interface::ControllerInterface)
