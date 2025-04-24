import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import math 

# TODO: transform points to cells function with tolerance
# TODO: publish occupancygrid representing cells that are hit by UV light

class SurfaceFilter(Node):
    def __init__(self):
        super().__init__('surface_filter')

        self.declare_parameter('angle_min', -np.pi)
        self.declare_parameter('angle_max', np.pi)
        self.declare_parameter('distance_max', 10.0) 
        self.declare_parameter('tolerance_map_hit', 0.1)
        self.declare_parameter('frame_scan_transform', 'uv_lamp_link') 

        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.distance_max = self.get_parameter('distance_max').value
        self.tolerance_map_hit = self.get_parameter('tolerance_map_hit').value
        self.frame_scan_transform = self.get_parameter('frame_scan_transform').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_array = None
        self.map_info = None

        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan', 
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

        self.subscription_map_update = self.create_subscription(
            OccupancyGridUpdate,
            '/map_update',
            self.map_update_callback,
            10
        )

        self.publisher_uv_map = self.create_publisher(OccupancyGrid, '/uv_exposure_map', 10)

        self.get_logger().info(f"SurfaceFilter node started. Transforming scans to frame: '{self.frame_scan_transform}'")

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Map received.")

    def map_update_callback(self, msg: OccupancyGridUpdate):
        if self.map_array is None:
            return
        patch = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
        self.map_array[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch

    def scan_callback(self, msg: LaserScan):
        tf = self.lookup_transform(msg)
        if tf is None:
            return None

        points = self.transform_points(msg, tf)
        points = self.filter_by_angle(points)
        points = self.filter_by_distance(points)
        #points = self.filter_by_map(points)

        transformed_scan = self.build_scan(msg, points)

        self.publisher_transformed_scan.publish(transformed_scan)
    
    def lookup_transform(self, scan_in: LaserScan):
        source_frame = scan_in.header.frame_id
        target_frame = self.frame_scan_transform
        scan_time = scan_in.header.stamp
        timeout_duration = rclpy.duration.Duration(seconds=0.1)

        if not self.tf_buffer.can_transform(target_frame, source_frame, scan_time, timeout=timeout_duration):
            self.get_logger().warn(f"Could not transform '{source_frame}' to '{target_frame}' at time {scan_time.sec}.{scan_time.nanosec}. Skipping scan.", throttle_duration_sec=5.0)
            return None

        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, scan_time, timeout=timeout_duration)
        except TransformException as ex:
            self.get_logger().error(f"Transform lookup failed: {ex}")
            return None

    def transform_points(self, scan_in: LaserScan, tf):
        q = tf.transform.rotation
        t = tf.transform.translation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        tx, ty = t.x, t.y

        num_points = len(scan_in.ranges)
        angles = scan_in.angle_min + np.arange(num_points) * scan_in.angle_increment
        ranges = np.array(scan_in.ranges)
        ranges[~np.isfinite(ranges)] = 0.0
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.stack([x, y], axis=1) @ R.T + np.array([tx, ty])

    def filter_by_angle(self, points: np.ndarray) -> np.ndarray:
        angles = np.arctan2(points[:, 1], points[:, 0])
        return points[(angles >= self.angle_min) & (angles <= self.angle_max)]
    
    def filter_by_distance(self, points: np.ndarray) -> np.ndarray:
        dists = np.linalg.norm(points, axis=1)
        return points[dists <= self.distance_max]

    def filter_by_map(self, points: np.ndarray) -> np.ndarray:
        if self.map_array is None or self.map_info is None:
            return np.empty((0, 2))

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        height, width = self.map_array.shape

        indices = ((points - [ox, oy]) / res).astype(int)

        valid_mask = (
            (indices[:, 0] >= 0) & (indices[:, 0] < width) &
            (indices[:, 1] >= 0) & (indices[:, 1] < height)
        )

        indices = indices[valid_mask]
        points = points[valid_mask]

        radius_cells = int(np.ceil(self.tolerance_map_hit / res))
        dx, dy = np.meshgrid(np.arange(-radius_cells, radius_cells + 1),
                            np.arange(-radius_cells, radius_cells + 1))
        offsets = np.stack((dx.ravel(), dy.ravel()), axis=1)
        dists = np.sqrt((offsets**2).sum(axis=1)) * res
        offsets = offsets[dists <= self.tolerance_map_hit]

        # Expand to (num_points, num_offsets, 2)
        expanded_indices = indices[:, np.newaxis, :] + offsets[np.newaxis, :, :]
        flat_indices = expanded_indices.reshape(-1, 2)

        # Clamp to inside bounds
        in_bounds = (
            (flat_indices[:, 0] >= 0) & (flat_indices[:, 0] < width) &
            (flat_indices[:, 1] >= 0) & (flat_indices[:, 1] < height)
        )
        flat_indices = flat_indices[in_bounds]

        # Convert (x, y) to map[y, x] index
        map_hits = np.zeros(len(points), dtype=bool)
        x_idx = flat_indices[:, 0]
        y_idx = flat_indices[:, 1]
        occupied_mask = self.map_array[y_idx, x_idx] == 100

        # Reverse lookup: which point each flat index came from
        num_offsets = len(offsets)
        point_indices = np.repeat(np.arange(len(points)), num_offsets)[in_bounds]

        # Mark points that have at least one nearby occupied cell
        map_hits[np.unique(point_indices[occupied_mask])] = True

        return points[map_hits]


    def publish_uv_coverage_map(self, uv_points: np.ndarray):
        if self.map_array is None or self.map_info is None or len(uv_points) == 0:
            return

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        height, width = self.map_array.shape

        # Convert points to map indices
        indices = ((uv_points - [ox, oy]) / res).astype(int)

        # Clamp to map bounds
        indices = indices[
            (indices[:, 0] >= 0) & (indices[:, 0] < width) &
            (indices[:, 1] >= 0) & (indices[:, 1] < height)
        ]

        if len(indices) == 0:
            return

        x_min, y_min = np.min(indices, axis=0)
        x_max, y_max = np.max(indices, axis=0)
        w = x_max - x_min + 1
        h = y_max - y_min + 1

        cropped_array = self.map_array[y_min:y_min + h, x_min:x_min + w]

        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import Header
        from geometry_msgs.msg import Pose

        cropped_map = OccupancyGrid()
        cropped_map.header = Header()
        cropped_map.header.stamp = self.get_clock().now().to_msg()
        cropped_map.header.frame_id = "map"
        cropped_map.info.resolution = res
        cropped_map.info.width = w
        cropped_map.info.height = h
        cropped_map.info.origin = Pose()
        cropped_map.info.origin.position.x = ox + x_min * res
        cropped_map.info.origin.position.y = oy + y_min * res
        cropped_map.info.origin.position.z = 0.0
        cropped_map.info.origin.orientation.w = 1.0

        cropped_map.data = cropped_array.flatten().tolist()
        self.uv_map_pub.publish(cropped_map)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
             node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()