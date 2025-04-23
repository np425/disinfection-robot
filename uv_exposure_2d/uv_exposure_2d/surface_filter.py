import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np


class SurfaceFilter(Node):
    def __init__(self):
        super().__init__('surface_filter')

        self.declare_parameter('min_angle', -np.pi)
        self.declare_parameter('max_angle', np.pi)
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('map_hit_tolerance', 0.1)
        self.declare_parameter('scan_transform_frame', 'base_link')

        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.max_distance = self.get_parameter('max_distance').value
        self.map_hit_tolerance = self.get_parameter('map_hit_tolerance').value
        self.scan_transform_frame = self.get_parameter('scan_transform_frame').value

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

        self.publisher = self.create_publisher(
            LaserScan,
            '/surface_scan',
            10)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def map_update_callback(self, msg):
        if self.map_array is None:
            return
        patch = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
        self.map_array[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        count = len(ranges)

        angles = angle_min + angle_increment * np.arange(count)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        points_scan = np.stack((x, y), axis=-1)

        points_transformed = self.transform_points(
            points_scan,
            scan_frame=msg.header.frame_id,
            target_frame=self.scan_transform_frame,
            stamp=msg.header.stamp
        )

        if points_transformed is None:
            return

        self.publish_transformed_scan(msg, points_transformed)
        
    def transform_points(self, points_scan, scan_frame, target_frame, stamp):
        # TODO: Make lamp rotated to the side 
        # TODO: Fix translation issues
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                scan_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        # Convert quaternion to yaw (2D rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rot = np.array([[cos_yaw, -sin_yaw],
                        [sin_yaw,  cos_yaw]])

        trans = np.array([t.x, t.y])

        return (rot @ points_scan.T).T + trans

    def publish_transformed_scan(self, msg, points_transformed):
        ranges_transformed = np.linalg.norm(points_transformed, axis=1)

        new_ranges = np.full_like(np.array(msg.ranges), np.inf)
        new_ranges[:len(ranges_transformed)] = ranges_transformed  

        scan_out = LaserScan()
        scan_out.header.stamp = msg.header.stamp
        scan_out.header.frame_id = self.scan_transform_frame
        scan_out.angle_min = msg.angle_min
        scan_out.angle_max = msg.angle_max
        scan_out.angle_increment = msg.angle_increment
        scan_out.time_increment = msg.time_increment
        scan_out.scan_time = msg.scan_time
        scan_out.range_min = msg.range_min
        scan_out.range_max = msg.range_max
        scan_out.ranges = new_ranges.tolist()
        scan_out.intensities = []  # add intensity mapping if needed

        self.publisher.publish(scan_out)

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
