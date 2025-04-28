#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Ensure included

// Add [[maybe_unused]] attribute for unused parameters if using C++17 or later
// #define MAYBE_UNUSED [[maybe_unused]]
// Otherwise, use the comment style /*parameter_name*/

namespace uv_waypoint_executor {

class UVLampExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  UVLampExecutor()
  : uv_lamp_on_duration_(0.0),
    is_enabled_(true)
    // No logger or clock initialization needed here anymore
    // node_ is a SharedPtr, default initialized to nullptr is fine.
  {
    // Constructor body
  }

  // Destructor
  ~UVLampExecutor() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node in UVLampExecutor plugin!"};
    }

    // *** Store the node handle ***
    node_ = node;

    // *** Use node_->get_logger() directly for logging ***
    RCLCPP_INFO(node_->get_logger(), "Initializing UVLampExecutor plugin: %s", plugin_name.c_str());

    // Declare and get parameters
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".uv_lamp_on_duration", rclcpp::ParameterValue(5.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name + ".enabled", rclcpp::ParameterValue(true));

    node_->get_parameter(plugin_name + ".uv_lamp_on_duration", uv_lamp_on_duration_);
    node_->get_parameter(plugin_name + ".enabled", is_enabled_);

    RCLCPP_INFO(node_->get_logger(), "UVLampExecutor params: enabled: %s, duration: %.2f",
        is_enabled_ ? "true" : "false", uv_lamp_on_duration_);

    if (uv_lamp_on_duration_ <= 0.0) {
      is_enabled_ = false;
      RCLCPP_INFO(node_->get_logger(), "UV lamp duration is zero or negative, disabling UVLampExecutor plugin.");
    } else if (!is_enabled_) {
      RCLCPP_INFO(node_->get_logger(), "UVLampExecutor plugin is explicitly disabled via parameter.");
    }

    // Create publisher using the stored node_
    publisher_ = node_->create_publisher<std_msgs::msg::Bool>("cmd_uv", 10);
    RCLCPP_INFO(node_->get_logger(), "UVLampExecutor initialized successfully.");
  }

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & /*pose*/, // Mark unused param
    const int & curr_waypoint_index) override
  {
    // Safety check: Ensure node_ is valid before proceeding
    if (!node_) {
        // Cannot log here easily without a valid logger
        std::cerr << "ERROR: node_ handle is null in processAtWaypoint!" << std::endl;
        return false; // Indicate critical failure
    }

    if (!is_enabled_) {
      RCLCPP_DEBUG(node_->get_logger(), "UVLampExecutor skipping waypoint %d (disabled).", curr_waypoint_index);
      return true;
    }

    // *** Use node_->get_logger() directly ***
    RCLCPP_INFO(
        node_->get_logger(),
        "At waypoint %d, enabling UV lamp for %.2f seconds",
        curr_waypoint_index, uv_lamp_on_duration_);

    // Ensure publisher is valid before using (good practice)
    if (!publisher_) {
        RCLCPP_ERROR(node_->get_logger(), "Publisher is not initialized!");
        return false; // Indicate failure
    }

    try {
        sendUVCommand(true);
        // *** Use node_->get_clock() directly ***
        node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(uv_lamp_on_duration_));
        sendUVCommand(false);
        RCLCPP_INFO(node_->get_logger(), "Waypoint %d task completed (UV lamp cycle).", curr_waypoint_index);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception during waypoint processing: %s", e.what());
        // Attempt to turn off lamp just in case it was left on
        try {
            sendUVCommand(false);
        } catch(...) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send final UV OFF command after exception.");
        }
        return false; // Indicate failure
    }

    return true; // Indicate success
  }

private:
  // *** Declare ONLY the necessary member variables ***
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_; // Store the node handle IS necessary
  // rclcpp::Logger logger_;                        // Removed
  // rclcpp::Clock::SharedPtr clock_;               // Removed
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  double uv_lamp_on_duration_;
  bool is_enabled_;

  void sendUVCommand(bool enable)
  {
    // Safety check: Ensure node_ is valid before proceeding
    if (!node_) {
        // Cannot log here easily without a valid logger
        throw std::runtime_error("sendUVCommand called when node_ handle is null!");
    }
    // Add check for publisher validity before publishing
    if (!publisher_) {
         throw std::runtime_error("sendUVCommand called before publisher was initialized!");
    }

    std_msgs::msg::Bool msg;
    msg.data = enable;
    // *** Use node_->get_logger() directly ***
    RCLCPP_INFO(node_->get_logger(), "Sending UV command: %s", enable ? "ON" : "OFF");
    publisher_->publish(msg);
  }
};

}  // namespace uv_waypoint_executor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(uv_waypoint_executor::UVLampExecutor, nav2_core::WaypointTaskExecutor)