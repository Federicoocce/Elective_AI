#!/usr/bin/env python3

import rclpy
import sys
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class Navigator(Node):
    def __init__(self):
        super().__init__('my_project_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return None

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_pose.pose.orientation = Quaternion(w=1.0)
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal: x={x}, y={y}")
        return self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()

    # Check for command-line arguments (sys.argv[0] is the script name)
    if len(sys.argv) < 3:
        navigator.get_logger().error("Usage: ros2 run my_project send_goal <x> <y>")
        return

    # Parse x and y from command line
    x = float(sys.argv[1])
    y = float(sys.argv[2])

    # Send goal and wait
    future = navigator.send_goal(x, y)
    rclpy.spin_until_future_complete(navigator, future)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()