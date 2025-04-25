import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from uv_exposure_msgs.msg import DoseGrid

class DoseToCostmap(Node):
    def __init__(self):
        super().__init__('dose_to_costmap')
        self.get_logger().info("Dose to Costmap node started.")

        self.declare_parameter('required_dose', 20.0)
        self.required_dose = self.get_parameter('required_dose').get_parameter_value().double_value

        self.pub_map = self.create_publisher(OccupancyGrid, '/dose_costmap', 10)
        self.create_subscription(DoseGrid, '/dose_grid', self.callback, 10)

    def callback(self, msg: DoseGrid):
        self.get_logger().info(f"Received DoseGrid with {len(msg.dose)} values.")

        out_map = OccupancyGrid()
        out_map.header = msg.header
        out_map.info = msg.info
        out_data = []

        for d in msg.dose:
            if d <= 0.0:
                out_data.append(-1)
            elif d >= self.required_dose:
                out_data.append(1)
            else:
                clamped_d = max(0.0, min(d, self.required_dose))
                scaled = int((clamped_d / self.required_dose) * 126) - 128
                out_data.append(scaled)

        out_map.data = out_data
        self.pub_map.publish(out_map)

def main():
    rclpy.init()
    rclpy.spin(DoseToCostmap())
    rclpy.shutdown()
