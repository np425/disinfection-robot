import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from uv_msgs.msg import UVStateStamped

class MockUVController(Node):
    def __init__(self):
        super().__init__('mock_uv_controller')
        self.uv_lamp_name = self.declare_parameter('uv_lamp_name', 'uv_lamp_link').value
        self.publish_rate = self.declare_parameter('publish_rate', 5.0).value
        self.declare_parameter('potential_power', 5.7)
        self.requested_uv_on = False
        self.last_uv_on = False
        self.uv_cmd_sub = self.create_subscription(Bool, 'cmd_uv', self.cmd_callback, 10)
        self.uv_state_pub = self.create_publisher(UVStateStamped, '/uv_state', 10)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_state)
        self.get_logger().info(f"Initialized with uv_lamp_name='{self.uv_lamp_name}', publish_rate={self.publish_rate} Hz")

    def cmd_callback(self, msg):
        self.requested_uv_on = msg.data

    def publish_state(self):
        if self.requested_uv_on != self.last_uv_on:
            self.get_logger().info(f"UV lamp state changed: {'ON' if self.requested_uv_on else 'OFF'}")
            self.last_uv_on = self.requested_uv_on
        potential_power = self.get_parameter('potential_power').get_parameter_value().double_value
        power_radiant = potential_power if self.requested_uv_on else 0.0
        msg = UVStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.uv_lamp_name
        msg.is_on = self.requested_uv_on
        msg.power_radiant = power_radiant
        self.uv_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockUVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
