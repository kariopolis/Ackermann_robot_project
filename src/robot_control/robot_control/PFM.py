import numpy as np
import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

# Topics
topic1 = "/SpeedControl"
topic3 = "/scan"


class ControllerNode(Node):
    def __init__(self, ka_u, kr_u, g_star, eps_control_u, jump_thresh, lookahead):

        super().__init__("controller_node")

        # ---- Physical offsets ----
        self.lidar_offset = math.pi / 2

        # ---- SAFETY PARAMS ----
        self.CRITICAL_DIST = 0.20
        self.BACKUP_SPEED  = -0.08
        self.BACKUP_STEER  = 12.0

        # ---- Car params ----
        self.lookahead = lookahead
        self.wheel_base = 0.17
        self.max_actual_angle = math.radians(35)

        self.upper_speed_limit_forwards = 0.20
        self.lower_speed_limit_forwards = 0.08

        # ---- Control params ----
        self.jump_thresh = jump_thresh
        self.ka = ka_u
        self.kr = kr_u
        self.g_star = g_star
        self.eps_goal = eps_control_u

        # ---- TF ----
        self.global_frame = "map"
        self.robot_frame = "base_link"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ---- State ----
        self.goal = None
        self.LidarMsg = LaserScan()

        # ---- Steering smoothing ----
        self.last_angle_cmd = 90.0            # deg
        self.max_steer_rate = 90.0            # deg/s
        self.dt = 0.1                         # controller period

        # ---- ROS ----
        self.create_subscription(Empty, "/go_forward", self.trigger_cb, 10)
        self.create_subscription(LaserScan, topic3, self.SensorCallbackLidar, 10)

        self.ControlPublisher = self.create_publisher(Float32MultiArray, topic1, 10)
        self.timer = self.create_timer(self.dt, self.ControlLoop)

    # ---------------------------------------------------------

    def trigger_cb(self, msg):

        pose = self.get_pose()
        if not pose:
            return

        x, y, theta = pose
        gx = x + self.lookahead * math.cos(theta)
        gy = y + self.lookahead * math.sin(theta)
        self.goal = (gx, gy)
        self.get_logger().info("Goal set")

    # ---------------------------------------------------------

    def get_pose(self):

        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        return t.x, t.y, yaw

    # ---------------------------------------------------------

    def SensorCallbackLidar(self, msg):
        self.LidarMsg = msg

    # ---------------------------------------------------------

    def orientationError(self, theta, thetaD):

        alpha = thetaD - theta
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        return alpha

    # ---------------------------------------------------------

    def ControlLoop(self):

        if not self.goal:
            self.publish(0.0, 90.0)
            return

        pose = self.get_pose()
        if not pose:
            return

        x, y, theta = pose
        xd, yd = self.goal

        ranges = np.array(self.LidarMsg.ranges)
        angle_min = self.LidarMsg.angle_min
        angle_inc = self.LidarMsg.angle_increment

        # -----------------------------------------------------
        # SAFETY BACKUP

        if ranges.size > 0:

            idx_all = np.arange(len(ranges))
            a_all = angle_min + idx_all * angle_inc + self.lidar_offset

            forward_mask = (np.abs(a_all) < math.radians(30.0))
            frontal = ranges[forward_mask]
            close = frontal[np.isfinite(frontal)]

            if close.size > 0 and np.min(close) < self.CRITICAL_DIST:

                side = np.sign(np.sum(a_all[forward_mask]))
                if side == 0:
                    side = 1.0

                steer = 90.0 + side * self.BACKUP_STEER

                self.publish(self.BACKUP_SPEED, steer)
                self.get_logger().warn("!!! SAFETY BACKUP !!!")
                return

        # -----------------------------------------------------
        # Attractive force

        vec = np.array([[x - xd], [y - yd]])
        AF = -self.ka * vec

        # -----------------------------------------------------
        # Repulsive force

        valid = (
            np.isfinite(ranges) &
            (ranges > 0.05) &
            (ranges < self.g_star)
        )

        RF = np.zeros((2, 1))

        if np.any(valid):

            idx = np.where(valid)[0]
            r = ranges[idx]

            laser_a = angle_min + idx * angle_inc
            a_rel = laser_a + self.lidar_offset

            a_world = a_rel + theta

            ox = x + r * np.cos(a_world)
            oy = y + r * np.sin(a_world)

            for a, px, py, d in zip(a_rel, ox, oy, r):

                sigma = math.radians(35.0)
                w = math.exp(-(a * a) / (2 * sigma * sigma))

                if abs(a) < math.radians(30.0):
                    w *= 2.0

                w = max(w, 0.05)

                pr = self.kr * ((1 / self.g_star) - (1 / d)) * (1 / (d ** 3))

                RF += w * pr * np.array([[x - px], [y - py]])

        RF = -RF

        # -----------------------------------------------------
        # Total force

        F = AF + RF

        thetaD = math.atan2(F[1, 0], F[0, 0])

        alpha = self.orientationError(theta, thetaD)
        alpha_deg = math.degrees(alpha)

        # -----------------------------------------------------
        # Goal reached

        if np.linalg.norm(vec) < self.eps_goal:
            self.publish(0.0, 90.0)
            self.get_logger().info("GOAL REACHED")
            return

        # -----------------------------------------------------
        # Steering

        max_wheel_deg = 35.0
        alpha_deg = max(-max_wheel_deg, min(max_wheel_deg, alpha_deg))

        angle_cmd = 90.0 - (alpha_deg / max_wheel_deg) * 90.0
        angle_cmd = max(0.0, min(180.0, angle_cmd))

        # -----------------------------------------------------
        # *** STEERING RATE LIMITER (SMOOTH TURNS) ***

        max_step = self.max_steer_rate * self.dt
        delta = angle_cmd - self.last_angle_cmd

        if abs(delta) > max_step:
            angle_cmd = self.last_angle_cmd + math.copysign(max_step, delta)

        self.last_angle_cmd = angle_cmd

        # -----------------------------------------------------
        # Speed

        turn_ratio = abs(alpha_deg) / max_wheel_deg

        speed = self.upper_speed_limit_forwards * (1.0 - 0.85 * turn_ratio)
        speed = max(speed, self.lower_speed_limit_forwards)

        Fmag = np.linalg.norm(F)
        scale = Fmag / (Fmag + 1.0)
        speed *= scale

        speed = min(speed, self.upper_speed_limit_forwards)

        # -----------------------------------------------------

        self.publish(speed, angle_cmd)

    # ---------------------------------------------------------

    def publish(self, speed, angle):

        msg = Float32MultiArray()
        msg.data = [float(speed), float(angle)]
        self.ControlPublisher.publish(msg)


# ============================================================

def main(args=None):
    rclpy.init()

    node = ControllerNode(
        ka_u=13,
        kr_u=0.6,
        g_star=1,
        eps_control_u=0.4,
        jump_thresh=0.4,
        lookahead=2.5,
    )

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
