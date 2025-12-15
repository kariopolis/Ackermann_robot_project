import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Subscribe to your controller odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.odom_callback,
            10
        )
        self.br = TransformBroadcaster(self)
        self.get_logger().info('Odom -> base_link TF broadcaster started')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        # Use whatever is in the message (you already see "odom" and "base_link")
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
