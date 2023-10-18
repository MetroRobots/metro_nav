from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from simple_actions.simple_client import SimpleActionClient, ResultCode
from nav2_msgs.action import ComputePathToPose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('global_planner_demo')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id = 'base_footprint'

        self.start = None
        self.goal = None

        self.action_client = SimpleActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.save_pose, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.save_goal, 1)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.transform)

    def save_pose(self, msg):
        pose = msg.pose.pose

        self.start = PoseStamped()
        self.start.header = msg.header
        self.start.pose = pose

        self.transform.transform.translation.x = pose.position.x
        self.transform.transform.translation.y = pose.position.y
        self.transform.transform.translation.z = pose.position.z

        self.transform.transform.rotation.x = pose.orientation.x
        self.transform.transform.rotation.y = pose.orientation.y
        self.transform.transform.rotation.z = pose.orientation.z
        self.transform.transform.rotation.w = pose.orientation.w

        self.plan()

    def save_goal(self, msg):
        self.goal = msg
        self.plan()

    def plan(self):
        if not self.start or not self.goal:
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start
        goal_msg.goal = self.goal
        goal_msg.use_start = True
        self.action_client.send_goal(goal_msg, self.done)

    def done(self, result_code, result):
        if result_code != ResultCode.SUCCEEDED:
            self.get_logger().warn('Planning failed.')
        else:
            d = result.planning_time.sec + result.planning_time.nanosec / 1e9
            self.get_logger().info(f'Found plan with {len(result.path.poses)} poses in {d:4f} seconds')


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
