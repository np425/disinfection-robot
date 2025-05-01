import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from uv_msgs.msg import UVStateStamped
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from datetime import datetime
import csv
import os
import math
import time

class UVPositionTracker(Node):
    def __init__(self):
        super().__init__('uv_position_tracker')
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.prev_state = None
        self.index = 0

        self.robot_link = self.declare_parameter('robot_link', 'base_link').value

        home_dir = os.path.expanduser('~')
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = os.path.join(home_dir, f'{now}_doses.csv')

        self.create_subscription(UVStateStamped, '/uv_state', self.uv_callback, 10)
        self.init_csv()

        self.get_logger().info(f"UVPositionTracker node started, saving to: {self.output_file}")

    def init_csv(self):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'index', 'state',
                'robot_x', 'robot_y', 'robot_yaw',
                'uv_x', 'uv_y', 'uv_yaw',
                'timestamp'
            ])

    def uv_callback(self, msg):
        state = 'ON' if msg.is_on else 'OFF'
        if self.prev_state is not None and msg.is_on == self.prev_state:
            return

        self.prev_state = msg.is_on

        try:
            robot_tf = self.buffer.lookup_transform('map', self.robot_link, rclpy.time.Time())
            uv_tf = self.buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        def extract_xy_yaw(tf: TransformStamped):
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw

        rx, ry, ryaw = extract_xy_yaw(robot_tf)
        ux, uy, uyaw = extract_xy_yaw(uv_tf)
        timestamp = time.time()

        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.index, state,
                rx, ry, ryaw,
                ux, uy, uyaw,
                f"{timestamp:.6f}"
            ])

        if state == 'OFF':
            self.index += 1

def main():
    rclpy.init()
    node = UVPositionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
