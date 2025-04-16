import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

WALL_START_COLOR = -128
# -128 not disinfected, -128-0 disinfection range, 1 disinfected

class MapModifierNode(Node):
    def __init__(self):
        super().__init__('map_modifier_node')
        self.pub_map = self.create_publisher(OccupancyGrid, '/map_custom', 10)
        self.map_data = None
        self.map_info = None
        self.wall_indices = []
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_timer(0.1, self.update_wall_colors)
        self.made_map = False

    def map_callback(self, msg):
        if self.made_map:
            return

        self.map_info = msg.info
        self.map_data = list(msg.data)  # Copy original map data

        self.wall_indices = []
        for i, val in enumerate(msg.data):
            if val == 100:
                self.map_data[i] = WALL_START_COLOR
                self.wall_indices.append(i)

        self.made_map = True

        self.get_logger().info(f'Found {len(self.wall_indices)} wall cells.')
        self.publish_map(self.make_header(msg.header.stamp))

    def update_wall_colors(self):
        if self.map_data is None or not self.wall_indices:
            return

        for i in self.wall_indices:
            val = self.map_data[i]
            self.map_data[i] = val + 1 if val < 127 else -128

        example_val = self.map_data[self.wall_indices[0]]
        self.get_logger().info(f'Current wall color: {example_val}')

        header = self.make_header(self.get_clock().now().to_msg())
        self.publish_map(header)

    def publish_map(self, header):
        out = OccupancyGrid()
        out.header = header
        out.info = self.map_info
        out.data = self.map_data
        self.pub_map.publish(out)

    def make_header(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = 'map'
        return header

def main():
    rclpy.init()
    rclpy.spin(MapModifierNode())
    rclpy.shutdown()
