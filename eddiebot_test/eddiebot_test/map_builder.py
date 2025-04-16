import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import tf2_ros
import tf_transformations


class MapDistanceVisualizer(Node):
    def __init__(self):
        super().__init__('map_distance_visualizer')
        self.ref_map = None
        self.ref_info = None
        self.latest_scan = None

        self.create_subscription(OccupancyGrid, '/map', self.ref_map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/map_with_distance', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.timer_callback)

    def ref_map_callback(self, msg):
        self.ref_info = msg.info
        self.ref_map = list(msg.data)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.ref_map is None or self.ref_info is None or self.latest_scan is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'rplidar_link', rclpy.time.Time())
            sensor_x = tf.transform.translation.x
            sensor_y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        except Exception as e:
            self.get_logger().warn(f'Could not get sensor pose: {e}')
            return

        width = self.ref_info.width
        height = self.ref_info.height
        res = self.ref_info.resolution
        ox = self.ref_info.origin.position.x
        oy = self.ref_info.origin.position.y

        output_map = [-1] * (width * height)

        for i, r in enumerate(self.latest_scan.ranges):
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                continue
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment

            # Point in sensor frame
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            # Rotate to map frame
            mx = math.cos(yaw) * lx - math.sin(yaw) * ly
            my = math.sin(yaw) * lx + math.cos(yaw) * ly

            # Translate to map frame
            x = sensor_x + mx
            y = sensor_y + my

            gx = int((x - ox) / res)
            gy = int((y - oy) / res)

            if not (0 <= gx < width and 0 <= gy < height):
                continue

            # Margin of error: check 3x3 neighborhood
            hit = False
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        idx = ny * width + nx
                        if self.ref_map[idx] == 100:
                            # mark that cell with distance-based value
                            wx = ox + nx * res
                            wy = oy + ny * res
                            dist = math.sqrt((wx - sensor_x) ** 2 + (wy - sensor_y) ** 2)
                            mapped_val = int((dist / 10.0) * 127) - 128
                            output_map[idx] = mapped_val
                            hit = True
                    if hit:
                        break
                if hit:
                    break

        self.publish_map(self.get_clock().now().to_msg(), output_map)

    def publish_map(self, stamp, map_data):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.info = self.ref_info
        grid.data = map_data
        self.pub.publish(grid)


def main():
    rclpy.init()
    rclpy.spin(MapDistanceVisualizer())
    rclpy.shutdown()
