import rclpy
from rclpy.node import Node
import math
from tf2_ros import TransformListener, Buffer
from std_srvs.srv import Empty, Trigger

class DistanceStatsNode(Node):
    def __init__(self):
        super().__init__('stats_distance')
        self.declare_parameter('move_threshold', 0.02)
        self.move_threshold = self.get_parameter('move_threshold').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1, self.timer_callback)
        self.last_pose = None
        self.total_distance = 0.0
        self.reset_srv = self.create_service(Empty, '/reset_distance', self.reset_callback)
        self.get_srv = self.create_service(Trigger, '/get_distance', self.get_callback)
        self.is_moving = False
        self.get_logger().info('Distance statistics are being measured')

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            if self.last_pose is not None:
                dx = x - self.last_pose[0]
                dy = y - self.last_pose[1]
                step_distance = math.hypot(dx, dy)

                if step_distance > self.move_threshold:
                    self.total_distance += step_distance
                    if not self.is_moving:
                        self.get_logger().info('Robot started moving')
                        self.is_moving = True
                else:
                    if self.is_moving:
                        self.get_logger().info('Robot stopped moving')
                        self.is_moving = False

            self.last_pose = (x, y)
        except Exception:
            pass

    def reset_callback(self, request, response):
        self.total_distance = 0.0
        self.last_pose = None
        self.is_moving = False
        self.get_logger().info('Distance reset')
        return response

    def get_callback(self, request, response):
        response.success = True
        response.message = f"Total distance traveled: {self.total_distance:.4f} meters"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DistanceStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
