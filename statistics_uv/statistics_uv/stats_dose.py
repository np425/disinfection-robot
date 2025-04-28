import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from uv_msgs.msg import DoseSparseGrid, DoseSparseGridUpdate
from std_srvs.srv import Trigger
import csv
import os
from datetime import datetime

# TODO: Separate into separate node which measures uv_state for total time

class DoseStatsNode(Node):
    def __init__(self):
        super().__init__('stats_dose')
        self.doses = {}

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.sub_grid = self.create_subscription(
            DoseSparseGrid,
            '/dose_grid',
            self.grid_callback,
            qos_transient
        )

        self.sub_update = self.create_subscription(
            DoseSparseGridUpdate,
            '/dose_grid_update',
            self.update_callback,
            10
        )

        self.save_srv = self.create_service(
            Trigger,
            '/save_dose_statistics',
            self.save_callback
        )

    def grid_callback(self, msg):
        for idx, dose in zip(msg.cell_indices, msg.doses):
            self.doses[idx] = dose

    def update_callback(self, msg):
        for idx, increment in zip(msg.cell_indices, msg.dose_increments):
            self.doses[idx] = self.doses.get(idx, 0.0) + increment

    def save_callback(self, request, response):
        try:
            home_dir = os.path.expanduser('~')
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(home_dir, f'{now}_doses.csv')

            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['cell_index', 'dose'])
                for idx, dose in self.doses.items():
                    writer.writerow([idx, dose])

            point_count = len(self.doses)
            self.get_logger().info(f"Saved {point_count} points into {filename}")

            response.success = True
            response.message = f"Saved {point_count} points to {filename}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoseStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
