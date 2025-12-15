import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import threading
import tkinter as tk
from tkinter import ttk


class AckermannSimControl(Node):
    def __init__(self):
        super().__init__("AckermannSimControl")

        # IMPORTANT: your controller expects TwistStamped
        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/ackermann_steering_controller/reference",
            10
        )

        self.speed = 0.0
        self.angle = 90.0

        self.timer = None

    def start(self):
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.publish_cmd)
            self.get_logger().info("Started publishing TwistStamped commands")

    def stop(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info("Stopped publishing")

    def publish_cmd(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Speed: -1..1
        msg.twist.linear.x = float(self.speed)

        # Steering: 90° = straight
        steering_rad = (float(self.angle) - 90.0) * 0.0174533
        msg.twist.angular.z = steering_rad

        self.publisher_.publish(msg)

    def emergency_stop(self):
        self.speed = 0.0
        self.publish_cmd()
        self.get_logger().info("EMERGENCY STOP")

    def straight_wheels(self):
        self.angle = 90.0
        self.publish_cmd()
        self.get_logger().info("Wheels straightened")


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = AckermannSimControl()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # ------------ GUI ------------
    root = tk.Tk()
    root.title("Ackermann Sim Controller")
    root.geometry("400x350")

    value_label = tk.Label(root, text="Speed: 0.0    Angle: 90°", font=("Arial", 12))
    value_label.pack(pady=10)

    def update_label():
        value_label.config(text=f"Speed: {node.speed:.2f}    Angle: {node.angle:.1f}°")
        root.after(100, update_label)

    update_label()

    tk.Label(root, text="Speed (-1 .. 1)").pack()
    speed_slider = ttk.Scale(
        root, from_=-1.0, to=1.0, orient="horizontal",
        command=lambda v: setattr(node, "speed", float(v))
    )
    speed_slider.set(0.0)
    speed_slider.pack(fill="x", padx=20)

    tk.Label(root, text="Wheel Angle (0° .. 180°)").pack()
    angle_slider = ttk.Scale(
        root, from_=0, to=180, orient="horizontal",
        command=lambda v: setattr(node, "angle", float(v))
    )
    angle_slider.set(90)
    angle_slider.pack(fill="x", padx=20)

    ttk.Button(root, text="Start Publishing", command=node.start).pack(pady=5)
    ttk.Button(root, text="Stop Publishing", command=node.stop).pack(pady=5)
    ttk.Button(root, text="STOP ROBOT", command=node.emergency_stop).pack(pady=5)
    ttk.Button(root, text="STRAIGHT WHEELS", command=node.straight_wheels).pack(pady=5)

    root.mainloop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
