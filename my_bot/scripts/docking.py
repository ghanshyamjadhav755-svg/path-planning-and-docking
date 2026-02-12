#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration


class Dock0Manager(Node):

    def __init__(self):
        super().__init__('dock0_manager')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('handover_distance', 0.45)
        self.declare_parameter('align_speed', 0.15)
        self.declare_parameter('angular_gain', 1.2)

        self.declare_parameter('blind_speed', 0.00)
        self.declare_parameter('blind_duration', 0.0)

        self.declare_parameter('undock_speed', -0.10)
        self.declare_parameter('undock_duration', 3.0)

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- PUB/SUB ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.command_sub = self.create_subscription(
            String,
            '/align_dock/command',
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/dock_status',
            10
        )

        # ---------------- STATE ----------------
        self.state = "IDLE"
        self.start_time = None

        # Optical frame
        self.camera_frame = "camera_link_optical"
        self.target_tag = "tag0"

        self.publish_status("IDLE")
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Dock0 Manager READY")

    # ------------------------------------------------
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ------------------------------------------------
    def command_callback(self, msg):

        cmd = msg.data.lower()

        if cmd == 'dock_0':
            self.state = "ALIGN"
            self.publish_status("DOCKING")

        elif cmd == 'undock':
            self.state = "UNDOCK"
            self.start_time = self.get_clock().now()
            self.publish_status("UNDOCKING")

        elif cmd == 'stop':
            self.state = "IDLE"
            self.cmd_pub.publish(Twist())
            self.publish_status("IDLE")

    # ------------------------------------------------
    def control_loop(self):

        cmd = Twist()

        # ================= IDLE =================
        if self.state == "IDLE":
            return

        # ================= UNDOCK =================
        if self.state == "UNDOCK":

            elapsed = (
                self.get_clock().now() - self.start_time
            ).nanoseconds * 1e-9

            if elapsed >= self.get_parameter('undock_duration').value:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                self.publish_status("UNDOCKED")
                return

            cmd.linear.x = self.get_parameter('undock_speed').value
            self.cmd_pub.publish(cmd)
            return

        # ================= ALIGN =================
        if self.state == "ALIGN":

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.target_tag,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
            except TransformException:
                # Tag not visible → stop
                self.cmd_pub.publish(Twist())
                return

            # Optical convention:
            # Z = forward
            # X = right
            # Y = down
            forward = tf.transform.translation.z
            lateral = tf.transform.translation.x

            # Proportional steering
            ang = -self.get_parameter('angular_gain').value * lateral
            ang = max(min(ang, 0.6), -0.6)

            cmd.linear.x = self.get_parameter('align_speed').value
            cmd.angular.z = ang

            # Handover to blind dock
            if forward <= self.get_parameter('handover_distance').value:
                self.state = "BLIND_DOCK"
                self.start_time = self.get_clock().now()
                return

            self.cmd_pub.publish(cmd)
            return

        # ================= BLIND DOCK =================
        if self.state == "BLIND_DOCK":

            elapsed = (
                self.get_clock().now() - self.start_time
            ).nanoseconds * 1e-9

            if elapsed >= self.get_parameter('blind_duration').value:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                self.publish_status("DOCKED")
                return

            cmd.linear.x = self.get_parameter('blind_speed').value
            self.cmd_pub.publish(cmd)
            return


# ====================================================
def main(args=None):
    rclpy.init(args=args)
    node = Dock0Manager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
