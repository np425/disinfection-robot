import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from uv_msgs.msg import UVStateStamped

class UVTimeTracker(Node):
    def __init__(self):
        super().__init__('uv_time_tracker')
        self.total_elapsed_time = 0.0
        self.last_update_time = None

        self.declare_parameter('timeout_threshold', 1.0)
        self.timeout_threshold = self.get_parameter('timeout_threshold').value

        self.sub_state = self.create_subscription(
            UVStateStamped,
            '/uv_state',
            self.state_callback,
            10
        )

        self.total_time_srv = self.create_service(
            Trigger,
            '/get_uv_time',
            self.get_total_time_callback
        )

        self.get_logger().info("UVTimeTracker node started")

    def state_callback(self, msg):
        now_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_update_time is not None:
            delta_t = now_time - self.last_update_time
            if delta_t <= self.timeout_threshold and msg.is_on:
                self.total_elapsed_time += delta_t

        self.last_update_time = now_time

    def get_total_time_callback(self, request, response):
        response.success = True
        response.message = f"Total disinfection time: {self.total_elapsed_time:.4f} seconds"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UVTimeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
