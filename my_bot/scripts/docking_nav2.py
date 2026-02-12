#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class DockingApplication(Node):

    def __init__(self):
        super().__init__('docking_application')

        # ---------------- ACTION CLIENT ----------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # ---------------- PUB/SUB ----------------
        self.align_pub = self.create_publisher(
            String,
            '/align_dock/command',
            10
        )

        self.create_subscription(
            String,
            'dock_control',
            self.control_callback,
            10
        )

        self.create_subscription(
            String,
            '/dock_status',
            self.dock_status_callback,
            10
        )

        # -------- DOCK APPROACH POSE (MAP FRAME) --------
        self.dock_pose = (
            1.0599023815564437,
            -0.6607276496476289,
            -0.0764395397098627,
            0.9970742182851506
        )

        self.mode = "IDLE"   # IDLE / NAVIGATING / ALIGNING / DOCKED / UNDOCKING

        self.get_logger().info("Docking Application Ready")

    # ------------------------------------------------
    def make_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        return pose

    # =================================================
    # USER CONTROL
    # =================================================
    def control_callback(self, msg):

        cmd = msg.data.lower()

        if cmd == "dock" and self.mode == "IDLE":
            self.start_navigation_to_dock()

        elif cmd == "undock" and self.mode == "DOCKED":
            self.start_undocking()

    # =================================================
    # NAVIGATE TO DOCK POSE
    # =================================================
    def start_navigation_to_dock(self):

        self.mode = "NAVIGATING"
        self.get_logger().info("Navigating to dock approach pose")

        self.nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(*self.dock_pose)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response)

    def nav_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Dock navigation rejected")
            self.mode = "IDLE"
            return

        goal_handle.get_result_async().add_done_callback(self.nav_result)

    def nav_result(self, future):

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached dock approach pose")

            self.mode = "ALIGNING"

            msg = String()
            msg.data = "dock_0"
            self.align_pub.publish(msg)

        else:
            self.get_logger().warn("Failed to reach dock pose")
            self.mode = "IDLE"

    # =================================================
    # DOCK STATUS FEEDBACK
    # =================================================
    def dock_status_callback(self, msg):

        status = msg.data

        if status == "DOCKED":
            self.get_logger().info("Docking successful")
            self.mode = "DOCKED"

        elif status == "UNDOCKED":
            self.get_logger().info("Undocking complete")
            self.mode = "IDLE"

    # =================================================
    # UNDOCK
    # =================================================
    def start_undocking(self):

        self.mode = "UNDOCKING"
        self.get_logger().info("Sending undock command")

        msg = String()
        msg.data = "undock"
        self.align_pub.publish(msg)


# ------------------------------------------------
def main():
    rclpy.init()
    node = DockingApplication()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
