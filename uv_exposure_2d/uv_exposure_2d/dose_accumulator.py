import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener

from std_srvs.srv import Empty

from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from uv_msgs.msg import UVStateStamped, DoseSparseGrid, DoseSparseGridUpdate
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, TransformStamped

import numpy as np
from typing import Dict, Tuple, Optional

from uv_common.tf_utils import lookup_tf, apply_tf_2d  


class DoseAccumulator(Node):
    def __init__(self) -> None:
        super().__init__('dose_accumulator')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timeout: float = self.declare_parameter('timeout', 1.0).get_parameter_value().double_value
        self.uv_frame: str = self.declare_parameter('uv_frame', 'uv_lamp_link').get_parameter_value().string_value

        self.dose_map: Dict[Tuple[int, int], float] = {}
        self.last_uv_state: Optional[UVStateStamped] = None
        self.last_uv_active_time: Optional[Time] = None
        self.last_dose_update_time: Optional[Time] = None

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=map_qos)
        self.sub_map_update = self.create_subscription(OccupancyGridUpdate, '/map_updates', self.map_update_callback, 10)
        self.sub_uv_state = self.create_subscription(UVStateStamped, '/uv_state', self.uv_state_callback, 10)
        self.sub_uv_exposure_map = self.create_subscription(OccupancyGridUpdate, '/uv_exposure_map', self.exposure_map_callback, 10)

        self.pub_dose_grid = self.create_publisher(DoseSparseGrid, '/dose_sparse_grid', qos_profile=map_qos)
        self.pub_dose_update = self.create_publisher(DoseSparseGridUpdate, '/dose_sparse_grid_updates', 10)

        self.srv_reset = self.create_service(Empty, '/reset', self.reset_callback)

        self.map_info: Optional[MapMetaData] = None
        self.map_header: Optional[Header] = None

        self.get_logger().info(f"DoseAccumulator node started")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_info = msg.info
        self.map_header = msg.header

        width = self.map_info.width
        data_np = np.array(msg.data, dtype=np.int8)
        wall_indices = np.flatnonzero(data_np == 100)
        i_coords = wall_indices % width
        j_coords = wall_indices // width

        new_keys = set(zip(i_coords, j_coords))
        for key in list(self.dose_map.keys()):
            if key not in new_keys:
                del self.dose_map[key]
        for key in new_keys:
            if key not in self.dose_map:
                self.dose_map[key] = 0.0

    def map_update_callback(self, msg: OccupancyGridUpdate) -> None:
        if self.map_info is None:
            return
        self.map_header = msg.header
        patch_width = msg.width
        patch_data = np.array(msg.data, dtype=np.int8)

        y_coords, x_coords = np.divmod(np.flatnonzero(patch_data == 100), patch_width)
        for x, y in zip(x_coords, y_coords):
            map_x = msg.x + x
            map_y = msg.y + y
            self.dose_map.setdefault((map_x, map_y), 0.0)

        y_coords, x_coords = np.divmod(np.flatnonzero(patch_data != 100), patch_width)
        for x, y in zip(x_coords, y_coords):
            map_x = msg.x + x
            map_y = msg.y + y
            self.dose_map.pop((map_x, map_y), None)

    def uv_state_callback(self, msg: UVStateStamped) -> None:
        self.last_uv_state = msg
        if msg.is_on:
            self.last_uv_active_time = Time.from_msg(msg.header.stamp)

    def exposure_map_callback(self, msg: OccupancyGridUpdate) -> None:
        if self.last_uv_state is None or not self.last_uv_state.is_on:
            return

        now = Time.from_msg(msg.header.stamp)
        dt_last_active = abs((now - self.last_uv_active_time).nanoseconds) * 1e-9
        if dt_last_active > self.timeout:
            self.get_logger().warn('UV lamp timed out, not accumulating dose.')
            return

        if self.map_header is None or self.map_info is None:
            return

        tf_uv_to_map = lookup_tf(self.tf_buffer, self.uv_frame, self.map_header.frame_id, rclpy.time.Time().to_msg())
        if tf_uv_to_map is None:
            self.get_logger().warn('Could not get UV frame to map transform.')
            return

        dt_dose = 0.0
        if self.last_dose_update_time:
            dt_dose = (now - self.last_dose_update_time).nanoseconds * 1e-9
        self.last_dose_update_time = now

        patch_data = np.array(msg.data, dtype=np.int8)
        wall_mask = (patch_data == 100)
        if not np.any(wall_mask):
            return

        y_patch, x_patch = np.divmod(np.flatnonzero(wall_mask), msg.width)
        x_global = msg.x + x_patch
        y_global = msg.y + y_patch
        wall_indices = np.stack((x_global, y_global), axis=-1)

        wall_xy = wall_indices_to_centers(wall_indices, self.map_info)

        lamp_xy = apply_tf_2d(np.zeros((1, 2), dtype=np.float32), tf_uv_to_map)[0]

        dists, cos_angles = compute_distances_and_angles(wall_xy, lamp_xy, tf_uv_to_map)
        doses = self.compute_dose(dists, cos_angles, dt_dose)

        self.update_dose_map(wall_indices, doses)

    def compute_dose(self, distances: np.ndarray, cos_angles: np.ndarray, dt_dose: float) -> np.ndarray:
        irradiance = self.last_uv_state.power_radiant / (4.0 * np.pi * np.maximum(distances**2, 1e-6))
        effective_cos = np.clip(cos_angles, 0.0, 1.0)
        dose = irradiance * effective_cos * dt_dose
        return dose
    
    def update_dose_map(self, wall_indices: np.ndarray, doses: np.ndarray) -> None:
        if self.map_info is None or self.map_header is None:
            return

        updated_entries: Dict[Tuple[int, int], float] = {}

        for (i, j), dose in zip(wall_indices, doses):
            key = (i, j)
            if key in self.dose_map:
                self.dose_map[key] += dose
                updated_entries[key] = dose
            else:
                self.get_logger().warn(f"Wall at ({i}, {j}) not found in dose_map, skipping.")

        if updated_entries:
            self.publish_update(updated_entries)

    def reset_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.dose_map.clear()
        self.publish_full_grid()
        self.get_logger().info('Dose map reset and published.')

        self.last_dose_update_time = None
        self.last_uv_state = None
        self.last_uv_active_time = None

        return response

    def publish_full_grid(self) -> None:
        if self.map_info is None or self.map_header is None:
            return

        msg = DoseSparseGrid()
        if self.last_dose_update_time is not None:
            msg.header.stamp = self.last_dose_update_time.to_msg()
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_header.frame_id
        msg.info = self.map_info

        cell_indices = []
        doses = []
        for (i, j), dose in self.dose_map.items():
            cell_indices.append(i + j * self.map_info.width)
            doses.append(dose)

        msg.cell_indices = cell_indices
        msg.doses = doses
        self.pub_dose_grid.publish(msg)

    def publish_update(self, updated_entries: Dict[Tuple[int, int], float]) -> None:
        if self.map_info is None or self.map_header is None:
            return

        msg = DoseSparseGridUpdate()
        if self.last_dose_update_time is not None:
            msg.header.stamp = self.last_dose_update_time.to_msg()
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = self.map_header.frame_id

        cell_indices = []
        dose_increments = []
        for (i, j), delta in updated_entries.items():
            cell_indices.append(i + j * self.map_info.width)
            dose_increments.append(delta)

        msg.cell_indices = cell_indices
        msg.dose_increments = dose_increments
        self.pub_dose_update.publish(msg)
        self.publish_full_grid()

def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def extract_wall_indices(msg: OccupancyGrid) -> np.ndarray:
    data_np = np.array(msg.data, dtype=np.int8)
    wall_indices = np.flatnonzero(data_np == 100)
    i_coords = wall_indices % msg.info.width
    j_coords = wall_indices // msg.info.width
    return np.stack((i_coords, j_coords), axis=-1)


def wall_indices_to_centers(wall_indices: np.ndarray, map_info: MapMetaData) -> np.ndarray:
    resolution = map_info.resolution
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y

    wall_xy = np.empty((wall_indices.shape[0], 2), dtype=np.float32)
    wall_xy[:, 0] = wall_indices[:, 0] * resolution + origin_x + resolution / 2.0
    wall_xy[:, 1] = wall_indices[:, 1] * resolution + origin_y + resolution / 2.0
    return wall_xy


def compute_distances_and_angles(wall2d: np.ndarray, lamp2d: np.ndarray, tf_uv_to_map: TransformStamped) -> Tuple[np.ndarray, np.ndarray]:
    diff = wall2d - lamp2d
    dists = np.linalg.norm(diff, axis=1)

    lamp_yaw = yaw_from_quaternion(tf_uv_to_map.transform.rotation)
    lamp_dir = np.array([np.cos(lamp_yaw), np.sin(lamp_yaw)], dtype=np.float32)

    diff_unit = diff / np.linalg.norm(diff, axis=1)[:, None]
    cos_angles = np.clip(np.sum(diff_unit * lamp_dir, axis=1), -1.0, 1.0)

    return dists, cos_angles


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoseAccumulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()