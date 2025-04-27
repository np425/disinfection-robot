import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from builtin_interfaces.msg import Time
import numpy as np
from scipy.ndimage import binary_dilation
from uv_common.tf_utils import lookup_tf, apply_tf_2d


class SurfaceFilter(Node):
    def __init__(self):
        super().__init__('surface_filter')

        self.declare_parameter('angle_min', -np.pi)
        self.declare_parameter('angle_max', np.pi)
        self.declare_parameter('distance_max', 2.0) 
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
        self.map_header = None

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
            '/map_updates',
            self.map_update_callback,
            10
        )

        self.pub_uv_map = self.create_publisher(OccupancyGridUpdate, '/uv_exposure_map', 10)
        self.pub_uv_scan = self.create_publisher(LaserScan, '/uv_exposure_scan', 10)

        self.get_logger().info(f"SurfaceFilter node started. Transforming scans to frame: '{self.frame_scan_transform}'")

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_header = msg.header
        self.get_logger().info("Map received.")

    def map_update_callback(self, msg: OccupancyGridUpdate):
        if self.map_array is None:
            return
        patch = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
        self.map_array[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch
        self.map_header = msg.header

    def scan_callback(self, msg: LaserScan):
        tf_scan_to_uv = lookup_tf(self.tf_buffer, msg.header.frame_id, self.frame_scan_transform, msg.header.stamp)
        if tf_scan_to_uv is None:
            return

        points_uv = self.scan_to_points(msg)
        points_uv = apply_tf_2d(points_uv, tf_scan_to_uv)
        points_uv = self.filter_by_angle(points_uv)
        points_uv = self.filter_by_distance(points_uv)

        out_scan = self.points_to_scan(points_uv, msg)
        self.pub_uv_scan.publish(out_scan)

        if self.map_header is None or self.map_info is None or self.map_array is None:
            return

        tf_uv_to_map = lookup_tf(self.tf_buffer, self.frame_scan_transform, self.map_header.frame_id, rclpy.time.Time().to_msg())
        if tf_uv_to_map is None:
            return

        points_map = apply_tf_2d(points_uv, tf_uv_to_map)

        roi, roi_offset = self.extract_roi(points_map, self.tolerance_map_hit)
        roi_surfaces = self.filter_by_map(points_map, roi, roi_offset)

        out_grid = self.roi_to_occupancygrid(roi_surfaces, roi_offset, msg.header.stamp)
        self.pub_uv_map.publish(out_grid)

    def scan_to_points(self, scan_in: LaserScan) -> np.ndarray:
        num_points = len(scan_in.ranges)
        angles = scan_in.angle_min + np.arange(num_points) * scan_in.angle_increment
        ranges = np.array(scan_in.ranges)
        ranges[~np.isfinite(ranges)] = 0.0
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.stack([x, y], axis=1)

    def filter_by_angle(self, points: np.ndarray) -> np.ndarray:
        angles = np.arctan2(points[:, 1], points[:, 0])
        return points[(angles >= self.angle_min) & (angles <= self.angle_max)]
    
    def filter_by_distance(self, points: np.ndarray) -> np.ndarray:
        dists = np.linalg.norm(points, axis=1)
        return points[dists <= self.distance_max]

    def filter_by_map(self, points_map_frame: np.ndarray, roi: np.ndarray, roi_offset: tuple[int, int]) -> np.ndarray:
        if self.map_info is None or roi.size == 0 or points_map_frame.size == 0:
            return np.zeros_like(roi, dtype=np.int8)

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        x_min, y_min = roi_offset
        height, width = roi.shape

        occupied_cells = (roi == 100)
        hit_radius_cells = int(np.ceil(self.tolerance_map_hit / res))
        hit_structure = np.ones((2 * hit_radius_cells + 1, 2 * hit_radius_cells + 1), dtype=bool)
        expanded_occupied_cells = binary_dilation(occupied_cells, structure=hit_structure)

        indices = ((points_map_frame - [ox, oy]) / res).astype(int)
        valid = (indices[:, 0] >= x_min) & (indices[:, 0] < x_min + width) & (indices[:, 1] >= y_min) & (indices[:, 1] < y_min + height)
        indices = indices[valid]
        shifted_indices = indices - np.array([x_min, y_min])
        hits = expanded_occupied_cells[shifted_indices[:, 1], shifted_indices[:, 0]]

        surface_hit_mask = np.zeros_like(roi, dtype=bool)
        surface_hit_mask[shifted_indices[hits, 1], shifted_indices[hits, 0]] = True
        surface_hit_mask_expanded = binary_dilation(surface_hit_mask, structure=hit_structure)

        final_surface_mask = surface_hit_mask_expanded & occupied_cells
        filtered_roi = np.where(final_surface_mask, 100, 0).astype(np.int8)

        return filtered_roi

    def extract_roi(self, points: np.ndarray, padding: float) -> tuple[np.ndarray, tuple[int, int]]:
        if self.map_array is None or self.map_info is None or points.shape[0] == 0:
            return np.zeros((0, 0), dtype=np.int8), (0, 0)

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        height, width = self.map_array.shape

        x_world_min = np.min(points[:, 0]) - padding
        x_world_max = np.max(points[:, 0]) + padding
        y_world_min = np.min(points[:, 1]) - padding
        y_world_max = np.max(points[:, 1]) + padding

        x_min = max(int((x_world_min - ox) / res), 0)
        x_max = min(int((x_world_max - ox) / res), width - 1)
        y_min = max(int((y_world_min - oy) / res), 0)
        y_max = min(int((y_world_max - oy) / res), height - 1)

        roi = self.map_array[y_min:y_max+1, x_min:x_max+1]
        roi_offset = (x_min, y_min)

        return roi, roi_offset

    def points_to_scan(self, points: np.ndarray, reference_scan: LaserScan) -> LaserScan:
        angles = np.arctan2(points[:, 1], points[:, 0])
        ranges = np.linalg.norm(points, axis=1)

        scan = LaserScan()
        scan.header.stamp = reference_scan.header.stamp  
        scan.header.frame_id = self.frame_scan_transform  
        scan.angle_min = reference_scan.angle_min
        scan.angle_max = reference_scan.angle_max
        scan.angle_increment = reference_scan.angle_increment
        scan.time_increment = reference_scan.time_increment
        scan.scan_time = reference_scan.scan_time
        scan.range_min = reference_scan.range_min
        scan.range_max = reference_scan.range_max

        num_ranges = int(np.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
        ranges_array = np.full(num_ranges, float('inf'), dtype=np.float32)

        valid = (angles >= scan.angle_min) & (angles <= scan.angle_max)
        if not np.any(valid):
            scan.ranges = ranges_array.tolist()
            return scan

        angles = angles[valid]
        ranges = ranges[valid]
        indices = ((angles - scan.angle_min) / scan.angle_increment).astype(np.int32)
        indices = np.clip(indices, 0, num_ranges - 1)

        np.minimum.at(ranges_array, indices, ranges)

        scan.ranges = ranges_array.tolist()
        return scan

    def roi_to_occupancygrid(self, roi: np.ndarray, roi_offset: tuple[int, int], stamp: Time) -> OccupancyGridUpdate:
        if self.map_array is None or self.map_info is None:
            return OccupancyGridUpdate()

        x_min, y_min = roi_offset

        update = OccupancyGridUpdate()
        update.header.frame_id = self.map_header.frame_id
        update.header.stamp = stamp
        update.x = x_min
        update.y = y_min
        update.width = roi.shape[1]
        update.height = roi.shape[0]
        update.data = roi.flatten().tolist()
        return update

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
