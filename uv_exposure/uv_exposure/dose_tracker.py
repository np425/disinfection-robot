import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from uv_exposure_msgs.msg import DoseGrid, UVStateStamped
import math
import tf2_ros
import tf_transformations

# TODO: timeout, lamp width and height outer bounds as points for disinfection, angle decay

class UVDoseTracker(Node):
    def __init__(self):
        super().__init__('uv_dose_tracker')
        self.get_logger().info("UV Dose Tracker node started.")

        self.declare_parameter('radiant_intensity', 1.6)
        self.declare_parameter('uv_frame', 'rplidar_link')
        self.radiant_intensity = self.get_parameter('radiant_intensity').get_parameter_value().double_value
        self.uv_frame = self.get_parameter('uv_frame').get_parameter_value().string_value

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(OccupancyGrid, '/map_update', self.map_update_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(UVStateStamped, '/uv_state', self.uv_state_callback, 10)
        self.pub = self.create_publisher(DoseGrid, '/dose_grid', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map = None
        self.map_info = None
        self.dose_data = None
        self.prev_scan_time = None
        self.uv_on = True

    def map_callback(self, msg):
        new_w = msg.info.width
        new_h = msg.info.height
        new_res = msg.info.resolution
        new_origin = msg.info.origin.position
        new_size = new_w * new_h

        if self.map_info and (
            new_w != self.map_info.width or
            new_h != self.map_info.height or
            new_res != self.map_info.resolution
        ):
            self.get_logger().warn("Map size/resolution changed. Remapping dose data.")
            new_dose = [0.0] * new_size

            old_w = self.map_info.width
            old_h = self.map_info.height
            old_res = self.map_info.resolution
            old_origin = self.map_info.origin.position

            for y in range(old_h):
                for x in range(old_w):
                    idx_old = y * old_w + x
                    dose = self.dose_data[idx_old]
                    if dose == 0.0:
                        continue

                    wx = old_origin.x + x * old_res
                    wy = old_origin.y + y * old_res
                    nx = int((wx - new_origin.x) / new_res)
                    ny = int((wy - new_origin.y) / new_res)
                    if 0 <= nx < new_w and 0 <= ny < new_h:
                        idx_new = ny * new_w + nx
                        new_dose[idx_new] = dose

            self.dose_data = new_dose
        elif not self.dose_data:
            self.dose_data = [0.0] * new_size

        self.map = list(msg.data)
        self.map_info = msg.info
        self.publish_dose_grid(msg.header.stamp)


    def map_update_callback(self, msg):
        if self.map is None:
            return
        invalidated = 0
        for y in range(msg.height):
            for x in range(msg.width):
                idx = (msg.y + y) * self.map_info.width + (msg.x + x)
                new_val = msg.data[y * msg.width + x]
                if self.map[idx] == 100 and new_val != 100:
                    self.dose_data[idx] = 0.0
                    invalidated += 1
                self.map[idx] = new_val
        if invalidated:
            self.get_logger().info(f"Map update: {invalidated} dose cells invalidated.")
        self.publish_dose_grid(msg.header.stamp)

    def uv_state_callback(self, msg):
        self.uv_on = msg.is_on
        state = "ON" if self.uv_on else "OFF"
        self.get_logger().info(f"UV {state}.")

    def scan_callback(self, msg):
        if not self.uv_on or self.map is None or self.map_info is None:
            return
        if self.prev_scan_time is None:
            self.prev_scan_time = Time.from_msg(msg.header.stamp)
            return

        now = Time.from_msg(msg.header.stamp)
        dt = (now - self.prev_scan_time).nanoseconds * 1e-9
        self.prev_scan_time = now

        try:
            sensor_x, sensor_y, yaw = self.get_sensor_pose(msg.header.frame_id)
            lamp_x, lamp_y = self.get_lamp_position()
        except Exception as e:
            self.get_logger().warn(f'TF error: {e}')
            return

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y = self.transform_lidar_point(r, angle, sensor_x, sensor_y, yaw)
            self.update_dose_grid(x, y, lamp_x, lamp_y, dt)

        self.publish_dose_grid(msg.header.stamp)

    def get_sensor_pose(self, frame_id):
        tf = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return x, y, yaw

    def get_lamp_position(self):
        tf = self.tf_buffer.lookup_transform('map', self.uv_frame, rclpy.time.Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def transform_lidar_point(self, r, angle, sx, sy, yaw):
        lx = r * math.cos(angle)
        ly = r * math.sin(angle)
        mx = math.cos(yaw) * lx - math.sin(yaw) * ly
        my = math.sin(yaw) * lx + math.cos(yaw) * ly
        return sx + mx, sy + my

    def update_dose_grid(self, x, y, lamp_x, lamp_y, dt):
        width = self.map_info.width
        height = self.map_info.height
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        if not (0 <= gx < width and 0 <= gy < height):
            return
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx = gx + dx
                ny = gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    idx = ny * width + nx
                    if self.map[idx] == 100:
                        wx = ox + nx * res
                        wy = oy + ny * res
                        dose = self.compute_dose(wx, wy, lamp_x, lamp_y, dt)
                        self.dose_data[idx] += dose

    def compute_dose(self, wx, wy, lamp_x, lamp_y, dt):
        dist_sq = (wx - lamp_x)**2 + (wy - lamp_y)**2
        if dist_sq == 0:
            return 0.0
        return (self.radiant_intensity / (2 * math.pi * dist_sq)) * 100 * dt

    def publish_dose_grid(self, stamp):
        grid = DoseGrid()
        grid.header = Header()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.info = self.map_info
        grid.time = float(self.get_clock().now().seconds_nanoseconds()[0])
        grid.dose = [float(d) for d in self.dose_data]
        self.pub.publish(grid)

def main():
    rclpy.init()
    rclpy.spin(UVDoseTracker())
    rclpy.shutdown()
