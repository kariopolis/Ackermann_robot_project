import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from sensor_msgs.msg import JointState

class JointsStates(Node):
    def __init__(self):
        super().__init__("joint_state_publisher_node")

        self.wheel_base = 0.17
        self.track_width = 0.175
        self.wheel_diameter = 0.065
        self.PPR = 330
        self.time_period = 0.1

        self.left_front_axis = 0.0
        self.right_front_axis = 0.0

        self.left_back_pos = 0.0
        self.right_back_pos = 0.0

        self.left_back_vel = 0.0
        self.right_back_vel = 0.0

        self.angle_sub = self.create_subscription(Float32MultiArray, "SpeedControl", self.get_angle, 10)
        self.speed_sub = self.create_subscription(Int16MultiArray, "CurrentSpeed", self.get_encoder_data, 10)

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(self.time_period, self.publish_js)

    def get_angle(self, msg):
        angle_deg_raw = (msg.data[1] - 90.0) / 90.0  # -1 .. 1
        max_virtual_deg = 35.0                       # between inner 31 and outer 40
        angle_deg = angle_deg_raw * max_virtual_deg
        steering_angle = math.radians(angle_deg)

        if abs(angle_deg) > 0.01:
            R = self.wheel_base / math.tan(steering_angle)
            half = self.track_width / 2
            R_in = R - half
            R_out = R + half

            if angle_deg <0:
                self.right_front_axis = math.atan(self.wheel_base / R_in)
                self.left_front_axis = math.atan(self.wheel_base / R_out)
            else:
                self.left_front_axis = math.atan(self.wheel_base / R_out)
                self.right_front_axis = math.atan(self.wheel_base / R_in)
        else:
            self.left_front_axis = 0.0
            self.right_front_axis = 0.0

    def get_encoder_data(self, msg):
        lt = msg.data[0]
        rt = msg.data[1]
        tot_l = msg.data[2]
        tot_r = msg.data[3]

        self.left_back_pos = tot_l * math.pi*2 / self.PPR
        self.right_back_pos = tot_r * math.pi*2 / self.PPR

        self.left_back_vel = lt * math.pi*2 / self.PPR / self.time_period
        self.right_back_vel = -rt * math.pi*2 / self.PPR / self.time_period

    def publish_js(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        js.name = [
            "left_front_axis_joint",
            "right_front_axis_joint",
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_back_wheel_joint",
            "right_back_wheel_joint"
        ]

        js.position = [
            -self.left_front_axis,
            -self.right_front_axis,
            0.0,
            0.0,
            self.left_back_pos,
            self.right_back_pos
        ]

        js.velocity = [
            0.0,
            0.0,
            0.0,
            0.0,
            self.left_back_vel,
            self.right_back_vel
        ]

        self.joint_pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(JointsStates())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
