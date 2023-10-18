from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class OdomToTF(Node):

    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 1)

    def odom_cb(self, msg):
        self.transform.header = msg.header
        self.transform.child_frame_id = msg.child_frame_id

        pose = msg.pose.pose
        self.transform.transform.translation.x = pose.position.x
        self.transform.transform.translation.y = pose.position.y
        self.transform.transform.translation.z = pose.position.z

        self.transform.transform.rotation.x = pose.orientation.x
        self.transform.transform.rotation.y = pose.orientation.y
        self.transform.transform.rotation.z = pose.orientation.z
        self.transform.transform.rotation.w = pose.orientation.w

        self.tf_broadcaster.sendTransform(self.transform)


def main():
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
