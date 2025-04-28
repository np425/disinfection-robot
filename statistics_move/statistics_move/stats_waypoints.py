import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import csv
import os
from datetime import datetime

class WaypointStatsNode(Node):
    def __init__(self):
        super().__init__('stats_waypoints')
        self.current_goal_pose = None
        self.waypoints = []

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.timer = self.create_timer(0.5, self.check_result_callback)

        self.save_srv = self.create_service(
            Trigger,
            '/save_waypoints',
            self.save_callback
        )

        self.last_goal_handle = None
        self.get_logger().info('Waypoint statistics are being measured')

    def goal_callback(self, msg):
        self.current_goal_pose = msg

    def check_result_callback(self):
        if not self.action_client.server_is_ready():
            return

        if self.action_client._goal_handles:
            # Take the last goal handle
            self.last_goal_handle = self.action_client._goal_handles[-1]

            if self.last_goal_handle.status == 4 or self.last_goal_handle.status == 5 or self.last_goal_handle.status == 6:
                self.record_waypoint(self.last_goal_handle.status)
                self.last_goal_handle = None

    def record_waypoint(self, status):
        if self.current_goal_pose is None:
            return

        pose = self.current_goal_pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.get_yaw_from_quaternion(pose.orientation)

        status_str = self.status_to_string(status)
        self.waypoints.append((x, y, yaw, status_str))
        self.get_logger().info(f"Waypoint recorded: x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}, status={status_str}")

        self.current_goal_pose = None

    def save_callback(self, request, response):
        try:
            home_dir = os.path.expanduser('~')
            now = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(home_dir, f'{now}_waypoints.csv')

            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'yaw', 'status'])
                for x, y, yaw, status_str in self.waypoints:
                    writer.writerow([f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}", status_str])

            self.get_logger().info(f"Saved {len(self.waypoints)} waypoints into {filename}")

            response.success = True
            response.message = f"Saved {len(self.waypoints)} waypoints to {filename}"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_yaw_from_quaternion(self, orientation):
        import math
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def status_to_string(self, status):
        if status == 4:
            return "SUCCEEDED"
        elif status == 5:
            return "CANCELED"
        elif status == 6:
            return "ABORTED"
        else:
            return "UNKNOWN"

def main(args=None):
    rclpy.init(args=args)
    node = WaypointStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
