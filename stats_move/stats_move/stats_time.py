import rclpy
from rclpy.node import Node
import math
from tf2_ros import TransformListener, Buffer
from std_srvs.srv import Empty, Trigger

class MovementTimeTracker(Node):
    def __init__(self):
        super().__init__('movement_time_tracker')
        self.declare_parameter('move_threshold', 0.005)
        self.move_threshold = self.get_parameter('move_threshold').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_pose = None
        self.last_time = None
        self.last_yaw = None
        self.total_movement_time = 0.0
        self.is_moving = False
        self.reset_srv = self.create_service(Empty, '/reset_movement_time', self.reset_callback)
        self.get_srv = self.create_service(Trigger, '/get_movement_time', self.get_callback)
        self.get_logger().info('Movement time statistics are being measured')

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            stamp = trans.header.stamp
            now = stamp.sec + stamp.nanosec * 1e-9

            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            if self.last_pose is not None and self.last_time is not None and self.last_yaw is not None:
                dx = x - self.last_pose[0]
                dy = y - self.last_pose[1]
                dpos = math.hypot(dx, dy)

                dyaw = abs(yaw - self.last_yaw)
                dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
                dyaw = abs(dyaw)

                dt = now - self.last_time

                if dpos > self.move_threshold or dyaw > 0.01:
                    self.total_movement_time += dt
                    if not self.is_moving:
                        self.get_logger().info('Robot started moving')
                        self.is_moving = True
                else:
                    if self.is_moving:
                        self.get_logger().info('Robot stopped moving')
                        self.is_moving = False

            self.last_pose = (x, y)
            self.last_yaw = yaw
            self.last_time = now
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=10.0)

    def reset_callback(self, request, response):
        self.total_movement_time = 0.0
        self.last_pose = None
        self.last_time = None
        self.is_moving = False
        self.get_logger().info('Movement time reset')
        return response

    def get_callback(self, request, response):
        response.success = True
        response.message = f"Total movement time: {self.total_movement_time:.4f} seconds"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MovementTimeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
