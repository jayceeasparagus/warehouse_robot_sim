import math
import random
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

StationPose = Tuple[float, float, float]

WAYPOINTS: Dict[str, StationPose] = {
    'A1': (-4.0, 0.75, -1.57),
    'A2': (0.0, 0.75, -1.57),
    'A3': (4.0, 0.75, -1.57),
    'B1': (-4.0, -0.75, 1.57),
    'B2': (0.0, -0.75, 1.57),
    'B3': (4.0, -0.75, 1.57),
}


@dataclass
class DeliveryJob:
    job_id: int
    pickup: str
    dropoff: str
    status: str = 'queued'


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class JobDispatcherNode(Node):
    def __init__(self):
        super().__init__('job_dispatcher_node')
        self.declare_parameter('job_interval_sec', 10.0)
        self.declare_parameter('max_jobs', 5)
        self.declare_parameter('pickup_wait_sec', 2.0)
        self.declare_parameter('seed', 0)

        self.job_interval_sec = float(self.get_parameter('job_interval_sec').value)
        self.max_jobs = int(self.get_parameter('max_jobs').value)
        self.pickup_wait_sec = float(self.get_parameter('pickup_wait_sec').value)
        seed = int(self.get_parameter('seed').value)
        self.random = random.Random(seed if seed != 0 else None)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.job_queue: Deque[DeliveryJob] = deque()
        self.next_job_id = 1
        self.completed_jobs = 0
        self.failed_jobs = 0

    def run(self):
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('Nav2 navigate_to_pose action server is not available.')
            return False

        self.get_logger().info(
            f'Dispatcher ready: job_interval={self.job_interval_sec:.1f}s, max_jobs={self.max_jobs}'
        )

        while rclpy.ok() and self.next_job_id <= self.max_jobs:
            job = self.generate_job()
            self.job_queue.append(job)
            self.get_logger().info(
                f'New job queued: job_id={job.job_id}, pickup={job.pickup}, '
                f'dropoff={job.dropoff}, queue_size={len(self.job_queue)}'
            )
            self.dispatch_next_job()

            if self.next_job_id <= self.max_jobs:
                self.get_logger().info(
                    f'Waiting {self.job_interval_sec:.1f}s before generating the next job.'
                )
                self.sleep_with_spin(self.job_interval_sec)

        self.get_logger().info(
            f'All jobs processed. Completed={self.completed_jobs}, Failed={self.failed_jobs}'
        )
        return self.failed_jobs == 0

    def generate_job(self) -> DeliveryJob:
        pickup, dropoff = self.random.sample(list(WAYPOINTS.keys()), 2)
        job = DeliveryJob(self.next_job_id, pickup, dropoff)
        self.next_job_id += 1
        return job

    def dispatch_next_job(self):
        if not self.job_queue:
            return

        job = self.job_queue.popleft()
        job.status = 'assigned'
        self.get_logger().info(
            f'Assigned job {job.job_id} to robot_1: {job.pickup} -> {job.dropoff}'
        )
        self.execute_job(job)

    def execute_job(self, job: DeliveryJob):
        job.status = 'going_to_pickup'
        if not self.navigate_to_station(job.pickup, 'pickup', job.job_id):
            self.mark_failed(job)
            return

        job.status = 'picking_up'
        self.get_logger().info(
            f'Job {job.job_id}: pickup complete at {job.pickup}; waiting {self.pickup_wait_sec:.1f}s'
        )
        self.sleep_with_spin(self.pickup_wait_sec)

        job.status = 'going_to_dropoff'
        if not self.navigate_to_station(job.dropoff, 'dropoff', job.job_id):
            self.mark_failed(job)
            return

        job.status = 'complete'
        self.completed_jobs += 1
        self.get_logger().info(
            f'Job {job.job_id} complete: {job.pickup} -> {job.dropoff}. '
            f'Completed={self.completed_jobs}, Failed={self.failed_jobs}'
        )

    def mark_failed(self, job: DeliveryJob):
        job.status = 'failed'
        self.failed_jobs += 1
        self.get_logger().error(
            f'Job {job.job_id} failed while executing {job.pickup} -> {job.dropoff}. '
            f'Completed={self.completed_jobs}, Failed={self.failed_jobs}'
        )

    def navigate_to_station(self, station: str, label: str, job_id: int) -> bool:
        pose = WAYPOINTS.get(station)
        if pose is None:
            self.get_logger().error(f'Job {job_id}: unknown {label} station {station}')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(*pose)
        x, y, yaw = pose
        self.get_logger().info(
            f'Job {job_id}: navigating to {label} {station} at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Job {job_id}: {label} goal {station} was rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error(f'Job {job_id}: {label} goal {station} returned no result')
            return False

        if result.status != 4:
            self.get_logger().error(
                f'Job {job_id}: {label} goal {station} failed with status {result.status}'
            )
            return False

        self.get_logger().info(f'Job {job_id}: reached {label} {station}')
        return True

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

    def sleep_with_spin(self, seconds: float):
        end_time = self.get_clock().now().nanoseconds + int(seconds * 1_000_000_000)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = JobDispatcherNode()
    try:
        success = node.run()
    except KeyboardInterrupt:
        success = True
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
