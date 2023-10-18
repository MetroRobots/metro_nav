from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node


from simple_actions.simple_client import SimpleActionClient, ResultCode
from nav2_msgs.action import ComputePathToPose, FollowPath


class SimpleExec(Node):

    def __init__(self):
        super().__init__('simple_executive')

        self.global_action = SimpleActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.local_action = SimpleActionClient(self, FollowPath, '/follow_path')

        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 1)

    def goal_cb(self, msg):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = msg
        self.global_action.send_goal(goal_msg, self.global_cb)

    def global_cb(self, result_code, result):
        if result_code != ResultCode.SUCCEEDED:
            self.get_logger().warn('Planning failed.')
            return

        d = result.planning_time.sec + result.planning_time.nanosec / 1e9
        self.get_logger().info(f'Found plan with {len(result.path.poses)} poses in {d:4f} seconds')

        goal_msg = FollowPath.Goal()
        goal_msg.path = result.path
        self.local_action.send_goal(goal_msg, self.local_cb)

    def local_cb(self, result_code, result):
        if result_code != ResultCode.SUCCEEDED:
            self.get_logger().warn('Control failed.')
            return
        self.get_logger().info('Goal Reached!')


def main():
    rclpy.init()
    node = SimpleExec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
