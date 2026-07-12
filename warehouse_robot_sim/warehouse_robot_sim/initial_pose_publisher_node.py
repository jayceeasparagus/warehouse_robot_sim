import math
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

RobotPose = Tuple[float, float, float]

INITIAL_POSES: Dict[str, RobotPose] = {
    'robot1': (-5.5, -1.0, 0.0),
    'robot2': (-5.5, 1.0, 0.0),
    'robot3': (-5.5, 0.0, 0.0),
}


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class InitialPosePublisherNode(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher_node')
        self.declare_parameter('start_delay_sec', 5.0)
        self.declare_parameter('publish_count', 8)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('require_subscribers', True)

        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.require_subscribers = bool(self.get_parameter('require_subscribers').value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.initial_pose_publishers = {
            robot_name: self.create_publisher(PoseWithCovarianceStamped, f'/{robot_name}/initialpose', qos)
            for robot_name in INITIAL_POSES
        }
        self.published = 0
        self.done = False
        self.waiting_for_subscribers = True
        self.timer = self.create_timer(self.start_delay_sec, self.start_publishing)
        self.get_logger().info(
            f'Waiting {self.start_delay_sec:.1f}s before publishing initial poses for '
            f'{", ".join(INITIAL_POSES)}'
        )

    def start_publishing(self):
        self.timer.cancel()
        self.timer = self.create_timer(self.publish_period_sec, self.publish_initial_poses)
        self.publish_initial_poses()

    def subscribers_ready(self) -> bool:
        missing = [
            robot_name
            for robot_name, publisher in self.initial_pose_publishers.items()
            if publisher.get_subscription_count() == 0
        ]
        if missing:
            self.get_logger().info(
                f'Waiting for AMCL initialpose subscribers: {", ".join(missing)}'
            )
            return False
        return True

    def publish_initial_poses(self):
        if self.require_subscribers and self.waiting_for_subscribers:
            if not self.subscribers_ready():
                return
            self.waiting_for_subscribers = False
            self.get_logger().info('AMCL initialpose subscribers are ready.')

        self.published += 1
        for robot_name, pose in INITIAL_POSES.items():
            msg = self.make_pose(*pose)
            self.initial_pose_publishers[robot_name].publish(msg)
            x, y, yaw = pose
            self.get_logger().info(
                f'Published initial pose {self.published}/{self.publish_count} for '
                f'{robot_name}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            )

        if self.published >= self.publish_count:
            self.get_logger().info('Initial pose publishing complete.')
            self.timer.cancel()
            self.done = True

    def make_pose(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg


def main():
    rclpy.init()
    node = InitialPosePublisherNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
