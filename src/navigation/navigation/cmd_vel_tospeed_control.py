import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

class CmdVelToSpeedControl(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_speed_control")

        self.wheel_base = 0.17
        self.max_steer_deg = 35.0
        self.max_linear_speed = 1.5

        self.sub = self.create_subscription(
            Twist, "cmd_vel_nav_filtered", self.cmd_vel_cb, 10
        )
        self.pub = self.create_publisher(
            Float32MultiArray, "SpeedControl", 10
        )

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = -msg.angular.z

        # --- Deadband ---
        if abs(v) < 0.01 and abs(w) < 0.01:
            self.publish_cmd(0.0, 90.0)
            return

        # --- Speed normalization ---
        speed_norm = max(-1.0, min(1.0, v / self.max_linear_speed))

        # --- Steering ---
        if abs(v) > 1e-3:
            delta = math.atan(self.wheel_base * w / v)
        else:
            delta = math.copysign(math.radians(self.max_steer_deg), w)

        # Reverse steering when backing up
        if v < 0:
            delta = -delta

        # Clamp steering
        delta_deg = math.degrees(delta)
        delta_deg = max(-self.max_steer_deg, min(self.max_steer_deg, delta_deg))

        # --- Map to servo ---
        servo_angle = 90.0 + (delta_deg / self.max_steer_deg) * 90.0
        servo_angle = max(0.0, min(180.0, servo_angle))

        self.publish_cmd(speed_norm, servo_angle)

    def publish_cmd(self, speed, servo):
        msg = Float32MultiArray()
        msg.data = [speed, servo]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSpeedControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
