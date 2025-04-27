import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class GoalStamper(Node):
    def __init__(self):
        super().__init__('goal_stamper')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sub = self.create_subscription(PoseStamped, '/rviz_goal', self.callback, 10)

    def callback(self, msg):
        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0
        msg.header.stamp = zero_time
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GoalStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
