import rclpy
import math

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q

class odometry_class(Node):
    def __init__(self):
        super().__init__("odometry_node")

        self.wheel_base = 0.17
        self.wheel_diameter = 0.065

        self.PPR = 330
        self.time_period = 0.1
        self.m_per_tick = (math.pi * self.wheel_diameter) / self.PPR

        # state
        self.v_left = 0.0
        self.v_right = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # subs/pubs
        self.angle_sub = self.create_subscription(
            Float32MultiArray, "SpeedControl", self.get_angle, 10)

        self.speed_sub = self.create_subscription(
            Int16MultiArray, "CurrentSpeed", self.get_encoder_data, 10)

        self.odom_pub = self.create_publisher(Odometry, "wheel/odom", 10)

        self.timer = self.create_timer(self.time_period, self.publish)

        self.steering_angle = 0.0

    def get_angle(self, msg):
        angle_deg_raw = (msg.data[1] - 90.0) / 90.0 #comes between 0 and 180
        self.steering_angle = -math.radians(angle_deg_raw * 35.0) #max trycicle steering angle is35

    def get_encoder_data(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        self.v_left = (left_ticks * self.m_per_tick) / self.time_period
        self.v_right = (right_ticks * self.m_per_tick) / self.time_period

    def publish(self):
        v_forward = (self.v_left + self.v_right) / 2.0
        yaw_rate = (v_forward / self.wheel_base) * math.tan(self.steering_angle)

        self.x += v_forward * math.cos(self.yaw) * self.time_period
        self.y += v_forward * math.sin(self.yaw) * self.time_period
        self.yaw += yaw_rate * self.time_period

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"          # RAW FRAME
        odom.child_frame_id = "base_link"      # no TF published

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v_forward
        odom.twist.twist.angular.z = yaw_rate

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = odometry_class()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
