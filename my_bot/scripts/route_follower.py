#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class NavigateLoop(Node):

    def __init__(self):
        super().__init__('navigate_loop')

        self.client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses'
        )

        # ===== ROUTE DEFINITION (name, x, y, z, w) =====
        self.route = [
            ('1',  0.2158270923694098,  3.9984935107647397,  0.030388678063095885,  0.9995381574735291),
            ('2', -6.0839847924473105,  3.9314760189298217, -0.7438426369822388,   0.6683547945569848),
            ('3',   -6.10204221958795,  -6.670928346411902, -0.014210700616816157,  0.9998990228958018),
            ('4',  8.87013657090683,  -6.5538847432090135, -0.9999971210621441,   0.0023995556721107853),
        ]
        # ==============================================

        self.forward = True
        self.send_route()

    def make_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        return pose

    def send_route(self):
        self.client.wait_for_server()

        path = self.route if self.forward else list(reversed(self.route))

        goal = NavigateThroughPoses.Goal()
        goal.poses = [
            self.make_pose(x, y, z, w)
            for _, x, y, z, w in path
        ]

        direction = 'FORWARD' if self.forward else 'REVERSE'
        self.get_logger().info(f'▶ Starting route ({direction})')

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Route rejected')
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Route completed')
        else:
            self.get_logger().warn(f'⚠ Route failed (status {status})')

        # Toggle direction and loop
        self.forward = not self.forward
        self.send_route()


def main():
    rclpy.init()
    node = NavigateLoop()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
