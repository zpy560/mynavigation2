import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class LocationPub(Node):
    def __init__(self):
        super().__init__('locationpub')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'turtlebot3_waffle')
        self.declare_parameter('x', 0.569)
        self.declare_parameter('y', 0.541)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_rate', 30.0)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than 0.0')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_tf)

        self.get_logger().info(
            f'Publishing localization TF {self.parent_frame} -> {self.child_frame} '
            f'at {self.x:.3f}, {self.y:.3f}, {self.z:.3f} yaw {self.yaw:.3f}')

    def publish_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LocationPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
