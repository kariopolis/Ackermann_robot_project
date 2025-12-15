
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import tkinter as tk
from tkinter import ttk
import math


class SpeedControlPublisher(Node):
    def __init__(self):
        super().__init__("robot_control_publisher")
        self.pub = self.create_publisher(Float32MultiArray, "SpeedControl", 10)
        self.timer = None
        self.speed = 0.0
        self.angle = 90.0

    def publish(self):
        msg = Float32MultiArray()
        msg.data = [self.speed, self.angle]
        self.pub.publish(msg)

    def start(self):
        if not self.timer:
            self.timer = self.create_timer(0.05, self.publish)  # 20 Hz
            self.get_logger().info("Joystick control STARTED")

    def stop(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.speed = 0.0
        self.angle = 90.0
        self.publish()
        self.get_logger().info("Joystick control STOPPED")

    def emergency_stop(self):
        self.speed = 0.0
        self.publish()

    def straighten(self):
        self.angle = 90.0
        self.publish()


# Joystick
class Joystick(tk.Canvas):
    def __init__(self, master, size=320):
        super().__init__(master, width=size, height=size, bg="#1e1e1e", highlightthickness=0)
        self.size = size
        self.r = size // 3
        cx = cy = size // 2
        self.create_oval(cx-self.r, cy-self.r, cx+self.r, cy+self.r, outline="#555", width=10)
        self.knob = self.create_oval(cx-40, cy-40, cx+40, cy+40, fill="#00ffff", outline="#00ffff", width=4)
        self.bind("<Button-1>", self.move)
        self.bind("<B1-Motion>", self.move)
        self.bind("<ButtonRelease-1>", self.release)
        self.callback = None
        self.release_callback = None

    def move(self, event):
        cx = self.size // 2
        dx = event.x - cx
        dy = event.y - cx
        dist = math.hypot(dx, dy)
        if dist > self.r:
            angle = math.atan2(dy, dx)
            dx = self.r * math.cos(angle)
            dy = self.r * math.sin(angle)
        x_norm = dx / self.r
        y_norm = -dy / self.r  # up = forward
        self.coords(self.knob, cx+dx-40, cx+dy-40, cx+dx+40, cx+dy+40)
        if self.callback:
            self.callback(x_norm, y_norm)

    def release(self, event=None):
        cx = self.size // 2
        self.coords(self.knob, cx-40, cx-40, cx+40, cx+40)
        if self.release_callback:
            self.release_callback()


def main():
    rclpy.init()
    node = SpeedControlPublisher()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title("Robot Joystick – Max Speed 0.5")
    root.geometry("460x680")
    root.configure(bg="#2b2b2b")

    # Value display
    label = tk.Label(root, text="Speed: 0.00 │ Angle: 90.0°", fg="#00ff00", bg="#2b2b2b", font=("Consolas", 16))
    label.pack(pady=20)

    # Joystick
    joy_frame = tk.Frame(root, bg="black")
    joy_frame.pack(pady=10)
    joy = Joystick(joy_frame, size=340)
    joy.pack()

    def on_move(x, y):
        node.speed = round(y * 0.5, 3)        # max ±0.5 m/s
        node.angle = round(90.0 + x * 90.0, 1)
        if abs(node.speed) < 0.03: node.speed = 0.0
        if abs(x) < 0.05: node.angle = 90.0
        label.config(text=f"Speed: {node.speed:+.2f} m/s │ Angle: {node.angle:.1f}°")
        node.publish()

    def on_release():
        node.speed = 0.0
        node.angle = 90.0
        label.config(text="Speed: 0.00 │ Angle: 90.0°")
        node.publish()

    joy.callback = on_move
    joy.release_callback = on_release

    # Buttons
    btn_frame = tk.Frame(root, bg="#2b2b2b")
    btn_frame.pack(pady=20)
    ttk.Button(btn_frame, text="START PUBLISHING", command=node.start).pack(fill="x", padx=50, pady=5)
    ttk.Button(btn_frame, text="STOP PUBLISHING", command=node.stop).pack(fill="x", padx=50, pady=5)
    ttk.Separator(root).pack(fill="x", pady=20, padx=50)
    ttk.Button(btn_frame, text="EMERGENCY STOP", command=node.emergency_stop).pack(fill="x", padx=50, pady=5)
    ttk.Button(btn_frame, text="STRAIGHT WHEELS", command=node.straighten).pack(fill="x", padx=50, pady=5)

    root.mainloop()
    node.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()