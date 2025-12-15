import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import math


class RelativeGoalNavigator(Node):
    def __init__(self):
        super().__init__("relative_goal_navigator")

        self.global_frame = "map"
        self.robot_frame  = "base_footprint"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        # Subscribe to forward distance command (meters)
        self.dist_sub = self.create_subscription(
            Float32, "go_forward_distance", self.dist_cb, 10
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.goal_active = False

        self.get_logger().info("Relative Goal Navigator ready")

    # -------------------------------------------------

    def dist_cb(self, msg):
        """Triggered by incoming distance command."""
        if self.goal_active:
            self.get_logger().warn("Goal already active - ignoring new command")
            return

        dist = msg.data

        # Safety clamp
        #dist = max(-3.0, min(3.0, dist))

        self.send_relative_goal(dist)

    # -------------------------------------------------

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except Exception:
            self.get_logger().warn("TF lookup failed")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )[2]

        return t.x, t.y, yaw

    # -------------------------------------------------

    def send_relative_goal(self, distance: float):

        pose = self.get_robot_pose()
        if not pose:
            return

        x, y, yaw = pose

        # Compute point ahead in map frame
        gx = x + distance * math.cos(yaw)
        gy = y + distance * math.sin(yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy

        # yaw → quaternion
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable")
            return

        self.get_logger().info(
            f"MAP RELATIVE GOAL => Δ={distance:.2f} m "
            f"(x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(yaw):.1f}°)"
        )

        self.goal_active = True

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)

    # -------------------------------------------------

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.goal_active = False
            self.get_logger().warn("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    # -------------------------------------------------

    def goal_result_cb(self, future):
        self.goal_active = False
        result = future.result().result

        self.get_logger().info(
            f"Goal completed with status: {result}"
        )

# -------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RelativeGoalNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
