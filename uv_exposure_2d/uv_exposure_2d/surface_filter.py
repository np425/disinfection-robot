import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import math 

class SurfaceFilter(Node):
    def __init__(self):
        super().__init__('surface_filter')

        # Default 'scan_transform_frame' should be the UV lamp's frame_id
        # e.g., 'uv_lamp_link'
        self.declare_parameter('min_angle', -np.pi)
        self.declare_parameter('max_angle', np.pi)
        self.declare_parameter('max_distance', 10.0) # Consider if this applies to original or transformed scan
        self.declare_parameter('map_hit_tolerance', 0.1)
        self.declare_parameter('scan_transform_frame', 'uv_lamp_link') # Example frame name

        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.max_distance = self.get_parameter('max_distance').value
        self.map_hit_tolerance = self.get_parameter('map_hit_tolerance').value
        self.scan_transform_frame = self.get_parameter('scan_transform_frame').value

        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_array = None
        self.map_info = None

        # --- Subscriptions ---
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan', # Original LiDAR scan topic
            self.scan_callback,
            10)

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )

        # Optional: Map updates (keep if needed for other filtering logic)
        self.subscription_map_update = self.create_subscription(
            OccupancyGridUpdate,
            '/map_update',
            self.map_update_callback,
            10
        )

        # --- Publisher ---
        # Publishes the scan as if it came from the UV lamp frame
        self.publisher_transformed_scan = self.create_publisher(
            LaserScan,
            '/surface_scan', # Topic for the transformed scan
            10)

        self.get_logger().info(f"SurfaceFilter node started. Transforming scans to frame: '{self.scan_transform_frame}'")

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Map received.")

    def map_update_callback(self, msg):
        if self.map_array is None:
            return
        patch = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
        self.map_array[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch

    def scan_callback(self, msg: LaserScan):
        transformed_scan = self.transform_scan(msg)
        if transformed_scan:
            self.publish_scan(transformed_scan)

    def transform_scan(self, scan_in: LaserScan) -> LaserScan | None:
        # TODO: Separate into multiple functions: transform -> filter angles -> filter distance -> filter map -> publish
        source_frame = scan_in.header.frame_id
        target_frame = self.scan_transform_frame
        scan_time = scan_in.header.stamp
        timeout_duration = rclpy.duration.Duration(seconds=0.1)

        if not self.tf_buffer.can_transform(target_frame, source_frame, scan_time, timeout=timeout_duration):
            self.get_logger().warn(
                f"Could not transform '{source_frame}' to '{target_frame}' at time {scan_time.sec}.{scan_time.nanosec}. Skipping scan.",
                throttle_duration_sec=5.0)
            return None

        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, scan_time, timeout=timeout_duration)
        except TransformException as ex:
            self.get_logger().error(f"Transform lookup failed: {ex}")
            return None

        # Extract translation and yaw rotation
        t = tf.transform.translation
        q = tf.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        tx, ty = t.x, t.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        R = np.array([[cos_yaw, -sin_yaw],
                    [sin_yaw,  cos_yaw]])

        # Prepare output scan
        scan_out = LaserScan()
        scan_out.header.stamp = scan_in.header.stamp
        scan_out.header.frame_id = target_frame
        scan_out.angle_min = scan_in.angle_min
        scan_out.angle_max = scan_in.angle_max
        scan_out.angle_increment = scan_in.angle_increment
        scan_out.time_increment = scan_in.time_increment
        scan_out.scan_time = scan_in.scan_time
        scan_out.range_min = scan_in.range_min
        scan_out.range_max = scan_in.range_max

        num_points = len(scan_in.ranges)
        scan_out.ranges = [float('inf')] * num_points

        # Convert to Cartesian in 2D
        angles = scan_in.angle_min + np.arange(num_points) * scan_in.angle_increment
        ranges = np.array(scan_in.ranges)
        ranges[~np.isfinite(ranges)] = 0.0
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Transform points in 2D
        points = np.stack([x, y], axis=1)
        transformed = points @ R.T + np.array([tx, ty])

        # Back to polar
        x_t = transformed[:, 0]
        y_t = transformed[:, 1]
        ranges_t = np.sqrt(x_t**2 + y_t**2)
        angles_t = np.arctan2(y_t, x_t)

        # Vectorized filtering: only valid angle indices
        epsilon = 1e-6
        valid_mask = (
            (angles_t >= scan_out.angle_min - epsilon) &
            (angles_t <= scan_out.angle_max + epsilon)
        )
        valid_angles = angles_t[valid_mask]
        valid_ranges = ranges_t[valid_mask]

        # Map angles to output indices
        indices = np.round((valid_angles - scan_out.angle_min) / scan_out.angle_increment).astype(int)

        # Clip to valid index range
        valid_idx_mask = (indices >= 0) & (indices < num_points)
        indices = indices[valid_idx_mask]
        valid_ranges = valid_ranges[valid_idx_mask]

        # For each index, store the minimum (closest) range
        current_ranges = np.array(scan_out.ranges)
        np.minimum.at(current_ranges, indices, valid_ranges)
        scan_out.ranges = current_ranges.tolist()

        return scan_out

    # --- Publishing Function ---
    def publish_scan(self, transformed_scan: LaserScan):
        """
        Publishes the transformed LaserScan message.

        Args:
            transformed_scan: The LaserScan message to publish.
        """
        if transformed_scan:
            self.publisher_transformed_scan.publish(transformed_scan)
        # else:
            # self.get_logger().debug("No valid transformed scan to publish.")


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure node is properly destroyed
        if rclpy.ok():
             node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()