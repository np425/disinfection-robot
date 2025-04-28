import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from uv_msgs.msg import DoseSparseGrid, DoseSparseGridUpdate
from std_srvs.srv import Trigger
import csv
import os
from datetime import datetime

class TimeStatsNode(Node):
    def __init__(self):
        super().__init__('stats_time')
        self.times = {}
        self.last_update_time = None
        self.total_elapsed_time = 0.0

        self.declare_parameter('timeout_threshold', 1.0)
        self.timeout_threshold = self.get_parameter('timeout_threshold').value

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
            '/save_exposure_times',
            self.save_callback
        )

        self.total_time_srv = self.create_service(
            Trigger,
            '/get_total_disinfection_time',
            self.get_total_time_callback
        )

    def grid_callback(self, msg):
        for idx in msg.cell_indices:
            self.times[idx] = 0.0

    def update_callback(self, msg):
        now_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_update_time is not None:
            delta_t = now_time - self.last_update_time
            if delta_t <= self.timeout_threshold:
                self.total_elapsed_time += delta_t
                for idx in msg.cell_indices:
                    if idx in self.times:
                        self.times[idx] += delta_t

        self.last_update_time = now_time

    def save_callback(self, request, response):
        try:
            home_dir = os.path.expanduser('~')
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(home_dir, f'{now}_times.csv')

            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['cell_index', 'exposure_time_sec'])
                for idx, time_sec in self.times.items():
                    writer.writerow([idx, f"{time_sec:.4f}"])

            point_count = len(self.times)
            self.get_logger().info(f"Saved {point_count} points into {filename}")

            response.success = True
            response.message = f"Saved {point_count} points to {filename}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_total_time_callback(self, request, response):
        response.success = True
        response.message = f"Total disinfection time: {self.total_elapsed_time:.4f} seconds"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TimeStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
