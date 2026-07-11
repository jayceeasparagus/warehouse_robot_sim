import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

StationPose = Tuple[float, float, float]

WAYPOINTS: Dict[str, StationPose] = {
    'A1': (-4.0, 1.25, -1.57),
    'A2': (0.0, 1.25, -1.57),
    'A3': (4.0, 1.25, -1.57),
    'B1': (-4.0, -1.25, 1.57),
    'B2': (0.0, -1.25, 1.57),
    'B3': (4.0, -1.25, 1.57),
}


@dataclass
class DeliveryJob:
    job_id: int
    pickup: str
    dropoff: str


@dataclass
class RobotWorker:
    name: str
    action_client: ActionClient
    busy: bool = False
    job: Optional[DeliveryJob] = None
    phase: str = 'idle'
    goal_handle: object = None


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class MultiRobotDispatcherNode(Node):
    def __init__(self):
        super().__init__('multi_robot_dispatcher_node')
        self.declare_parameter('robots', ['robot1', 'robot2'])
        self.declare_parameter('job_interval_sec', 6.0)
        self.declare_parameter('max_jobs', 6)
        self.declare_parameter('pickup_wait_sec', 2.0)
        self.declare_parameter('startup_wait_sec', 8.0)
        self.declare_parameter('seed', 0)

        robot_names = list(self.get_parameter('robots').value)
        self.job_interval_sec = float(self.get_parameter('job_interval_sec').value)
        self.max_jobs = int(self.get_parameter('max_jobs').value)
        self.pickup_wait_sec = float(self.get_parameter('pickup_wait_sec').value)
        self.startup_wait_sec = float(self.get_parameter('startup_wait_sec').value)
        seed = int(self.get_parameter('seed').value)
        self.random = random.Random(seed if seed != 0 else None)

        self.robots: List[RobotWorker] = [
            RobotWorker(name, ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose'))
            for name in robot_names
        ]
        self.localized_robots = set()
        self.amcl_subscriptions = [
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, robot_name=name: self.mark_robot_localized(robot_name),
                10,
            )
            for name in robot_names
        ]
        self.job_queue: List[DeliveryJob] = []
        self.next_job_id = 1
        self.generated_jobs = 0
        self.completed_jobs = 0
        self.failed_jobs = 0

        self.generator_timer = None
        self.dispatch_timer = None

    def run(self) -> bool:
        if not self.wait_for_robot_servers():
            return False

        self.get_logger().info(
            f'Multi-robot dispatcher ready: robots={self.robot_names()}, '
            f'job_interval={self.job_interval_sec:.1f}s, max_jobs={self.max_jobs}. '
            f'Waiting {self.startup_wait_sec:.1f}s for AMCL initial poses to settle.'
        )
        self.sleep_with_spin(self.startup_wait_sec)
        self.generator_timer = self.create_timer(self.job_interval_sec, self.generate_job_timer)
        self.dispatch_timer = self.create_timer(0.5, self.dispatch_jobs)

        self.generate_job_timer()

        while rclpy.ok() and not self.is_done():
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info(
            f'Dispatcher finished. Generated={self.generated_jobs}, '
            f'Completed={self.completed_jobs}, Failed={self.failed_jobs}'
        )
        return self.failed_jobs == 0

    def wait_for_robot_servers(self) -> bool:
        for robot in self.robots:
            self.get_logger().info(f'Waiting for /{robot.name}/navigate_to_pose action server...')
            if not robot.action_client.wait_for_server(timeout_sec=25.0):
                self.get_logger().error(f'/{robot.name}/navigate_to_pose is not available.')
                return False

        for robot in self.robots:
            if not self.wait_for_amcl_pose(robot.name):
                return False
        return True

    def mark_robot_localized(self, robot_name: str):
        self.localized_robots.add(robot_name)

    def wait_for_amcl_pose(self, robot_name: str, timeout_sec: float = 90.0) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1_000_000_000)
        self.get_logger().info(f'{robot_name}: waiting for /{robot_name}/amcl_pose...')

        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            if robot_name in self.localized_robots:
                self.get_logger().info(f'{robot_name}: AMCL pose received; localization is ready.')
                return True
            self.sleep_with_spin(0.5)

        self.get_logger().error(
            f'{robot_name}: timed out waiting for /{robot_name}/amcl_pose. '
            'Make sure initial poses are published after Nav2 is active.'
        )
        return False

    def robot_names(self) -> str:
        return ', '.join(robot.name for robot in self.robots)

    def is_done(self) -> bool:
        all_jobs_generated = self.generated_jobs >= self.max_jobs
        all_robots_idle = all(not robot.busy for robot in self.robots)
        return all_jobs_generated and not self.job_queue and all_robots_idle

    def generate_job_timer(self):
        if self.generated_jobs >= self.max_jobs:
            if self.generator_timer is not None:
                self.generator_timer.cancel()
            return

        pickup, dropoff = self.random.sample(list(WAYPOINTS.keys()), 2)
        job = DeliveryJob(self.next_job_id, pickup, dropoff)
        self.next_job_id += 1
        self.generated_jobs += 1
        self.job_queue.append(job)

        self.get_logger().info(
            f'Queued job {job.job_id}: {job.pickup} -> {job.dropoff}; '
            f'queue_size={len(self.job_queue)}'
        )

    def dispatch_jobs(self):
        for robot in self.robots:
            if robot.busy or not self.job_queue:
                continue

            job = self.job_queue.pop(0)
            robot.busy = True
            robot.job = job
            robot.phase = 'pickup'
            self.get_logger().info(
                f'Assigned job {job.job_id} to {robot.name}: {job.pickup} -> {job.dropoff}'
            )
            self.send_robot_to_station(robot, job.pickup)

    def send_robot_to_station(self, robot: RobotWorker, station: str):
        pose = WAYPOINTS[station]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(*pose)
        x, y, yaw = pose
        self.get_logger().info(
            f'{robot.name} job {robot.job.job_id}: going to {robot.phase} {station} '
            f'at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        future = robot.action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda done, r=robot, s=station: self.goal_response(done, r, s))

    def goal_response(self, future, robot: RobotWorker, station: str):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.fail_robot_job(robot, f'goal to {station} was rejected')
            return

        robot.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, r=robot, s=station: self.goal_result(done, r, s)
        )

    def goal_result(self, future, robot: RobotWorker, station: str):
        result = future.result()
        if result is None or result.status != 4:
            status = 'none' if result is None else result.status
            self.fail_robot_job(robot, f'goal to {station} failed with status {status}')
            return

        job = robot.job
        if robot.phase == 'pickup':
            self.get_logger().info(
                f'{robot.name} job {job.job_id}: pickup complete at {station}; '
                f'waiting {self.pickup_wait_sec:.1f}s'
            )
            robot.phase = 'dropoff_wait'
            wait_timer = None

            def finish_pickup_wait(r=robot):
                wait_timer.cancel()
                self.start_dropoff_once(r)

            wait_timer = self.create_timer(self.pickup_wait_sec, finish_pickup_wait)
            return

        if robot.phase == 'dropoff':
            self.completed_jobs += 1
            self.get_logger().info(
                f'{robot.name} job {job.job_id} complete: {job.pickup} -> {job.dropoff}. '
                f'Completed={self.completed_jobs}, Failed={self.failed_jobs}'
            )
            self.release_robot(robot)

    def start_dropoff_once(self, robot: RobotWorker):
        if robot.phase != 'dropoff_wait':
            return
        robot.phase = 'dropoff'
        self.send_robot_to_station(robot, robot.job.dropoff)

    def fail_robot_job(self, robot: RobotWorker, reason: str):
        job = robot.job
        self.failed_jobs += 1
        if job is None:
            self.get_logger().error(f'{robot.name}: failed without an assigned job: {reason}')
        else:
            self.get_logger().error(
                f'{robot.name} job {job.job_id} failed: {job.pickup} -> {job.dropoff}; {reason}. '
                f'Completed={self.completed_jobs}, Failed={self.failed_jobs}'
            )
        self.release_robot(robot)

    def release_robot(self, robot: RobotWorker):
        robot.busy = False
        robot.job = None
        robot.phase = 'idle'
        robot.goal_handle = None

    def sleep_with_spin(self, seconds: float):
        end_time = self.get_clock().now().nanoseconds + int(seconds * 1_000_000_000)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


def main():
    rclpy.init()
    node = MultiRobotDispatcherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        success = node.run()
    except KeyboardInterrupt:
        success = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
