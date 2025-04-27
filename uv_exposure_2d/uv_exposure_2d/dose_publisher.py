import rclpy
from rclpy.node import Node
from uv_msgs.msg import UVStateStamped

class UVPublisher(Node):
    def __init__(self):
        super().__init__('uv_publisher')
        self.pub = self.create_publisher(UVStateStamped, 'uv_state', 10)
        self.timer = self.create_timer(1, self.publish_msg)

    def publish_msg(self):
        msg = UVStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uv_lamp_link'
        msg.is_on = True
        msg.power_radiant = 5.37
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = UVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
