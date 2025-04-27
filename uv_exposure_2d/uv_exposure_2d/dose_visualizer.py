import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from uv_msgs.msg import DoseSparseGrid, DoseSparseGridUpdate

import numpy as np
from typing import Optional

class DoseVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('dose_visualizer')

        self.declare_parameter('dose_threshold', 100.0)
        self.dose_threshold = self.get_parameter('dose_threshold').get_parameter_value().double_value

        self.map_info: Optional[MapMetaData] = None
        self.map_header: Optional[Header] = None
        self.dose_array: Optional[np.ndarray] = None

        qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.sub_grid = self.create_subscription(DoseSparseGrid, '/dose_sparse_grid', self.grid_callback, qos_profile=qos_transient)
        self.sub_update = self.create_subscription(DoseSparseGridUpdate, '/dose_sparse_grid_updates', self.update_callback, 10)

        self.pub_heatmap = self.create_publisher(OccupancyGrid, '/dose_heatmap', qos_profile=qos_transient)
        self.pub_heatmap_update = self.create_publisher(OccupancyGridUpdate, '/dose_heatmap_updates', 10)

        self.get_logger().info("DoseVisualizer node started.")

    def grid_callback(self, msg: DoseSparseGrid) -> None:
        self.map_info = msg.info
        self.map_header = msg.header

        size = msg.info.width * msg.info.height
        self.dose_array = np.zeros(size, dtype=np.float32)
        self.dose_array[np.array(msg.cell_indices, dtype=np.uint32)] = msg.doses
        self.publish_full_grid()

    def update_callback(self, msg: DoseSparseGridUpdate) -> None:
        if self.dose_array is None:
            return

        self.map_header = msg.header
        self.dose_array[np.array(msg.cell_indices, dtype=np.uint32)] += msg.dose_increments
        self.publish_update(msg)

    def publish_full_grid(self) -> None:
        if self.map_info is None or self.map_header is None or self.dose_array is None:
            return

        msg = OccupancyGrid()
        msg.header = self.map_header
        msg.info = self.map_info

        scaled = np.clip((self.dose_array / self.dose_threshold) * 100, 0, 100) 
        msg.data = scaled.astype(np.int8).tolist()

        self.pub_heatmap.publish(msg)

    def publish_update(self, msg: DoseSparseGridUpdate) -> None:
        if self.map_info is None or self.map_header is None:
            return

        indices = np.array(msg.cell_indices, dtype=np.uint32)
        rows, cols = np.divmod(indices, self.map_info.width)

        min_row, max_row = rows.min(), rows.max()
        min_col, max_col = cols.min(), cols.max()

        patch_height = max_row - min_row + 1
        patch_width = max_col - min_col + 1

        update_msg = OccupancyGridUpdate()
        update_msg.header = msg.header
        update_msg.x = int(min_col)
        update_msg.y = int(min_row)
        update_msg.width = int(patch_width)
        update_msg.height = int(patch_height)

        full_array = self.dose_array.reshape((self.map_info.height, self.map_info.width))
        patch = full_array[min_row:max_row+1, min_col:max_col+1]

        scaled_patch = np.clip((patch / self.dose_threshold) * 100, 0, 100).astype(np.int8)

        update_msg.data = scaled_patch.ravel().tolist()

        self.pub_heatmap_update.publish(update_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoseVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
