from base2d_kinematics.kinematics_sub import KinematicsSub
from geometry_msgs.msg import Twist, TwistStamped


class CommandPublisher:
    def __init__(self, node, default_linear_speed=1.0, default_angular_speed=1.0):
        self.node = node
        self.node.declare_parameter('twist_topic', '/cmd_vel')
        self.node.declare_parameter('linear_speed', default_linear_speed)
        self.node.declare_parameter('angular_speed', default_angular_speed)
        self.node.declare_parameter('stamped_twists', False)
        self.node.declare_parameter('twist_frame', '')

        self.k_sub = KinematicsSub(self.node, self.kinematics_cb)

        topic = self.node.get_parameter('twist_topic').value
        self.stamped = self.node.get_parameter('stamped_twists').value
        if self.stamped:
            msg_type = TwistStamped
        else:
            msg_type = Twist
        self.velocity_publisher = self.node.create_publisher(msg_type, topic, 1)

    def kinematics_cb(self, msg):
        self.node.set_parameter('linear_speed', msg.max_speed_xy)
        self.node.set_parameter('angular_speed', msg.max_vel_theta)

    def publish_scaled_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                               angular_x=0.0, angular_y=0.0, angular_z=0.0):
        linear_speed = self.node.get_parameter('linear_speed').value
        angular_speed = self.node.get_parameter('angular_speed').value
        self.publish_command(linear_x * linear_speed, linear_y * linear_speed, linear_z * linear_speed,
                             angular_x * angular_speed, angular_y * angular_speed, angular_z * angular_speed)

    def publish_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.node.get_parameter('twist_frame').value
            twist = msg.twist
        else:
            twist = Twist()
            msg = twist
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z

        self.velocity_publisher.publish(msg)
