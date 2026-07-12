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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

StationPose = Tuple[float, float, float]

INITIAL_POSES: Dict[str, StationPose] = {
    'robot1': (-5.5, -1.0, 0.0),
    'robot2': (-5.5, 1.0, 0.0),
    'robot3': (-5.5, 0.0, 0.0),
}

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
    created_time: float
    attempts: int = 0

    @property
    def route(self) -> str:
        return f'{self.pickup}->{self.dropoff}'


@dataclass
class RobotWorker:
    name: str
    action_client: ActionClient
    busy: bool = False
    job: Optional[DeliveryJob] = None
    phase: str = 'idle'
    goal_handle: object = None
    last_station: Optional[str] = None


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def station_distance(a: str, b: str) -> float:
    ax, ay, _ = WAYPOINTS[a]
    bx, by, _ = WAYPOINTS[b]
    return math.hypot(ax - bx, ay - by)


def pose_to_station_distance(pose: StationPose, station: str) -> float:
    px, py, _ = pose
    sx, sy, _ = WAYPOINTS[station]
    return math.hypot(px - sx, py - sy)


class MultiRobotDispatcherNode(Node):
    def __init__(self):
        super().__init__('multi_robot_dispatcher_node')
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('job_interval_sec', 4.0)
        self.declare_parameter('max_jobs', 9)
        self.declare_parameter('pickup_wait_sec', 1.5)
        self.declare_parameter('assignment_stagger_sec', 1.0)
        self.declare_parameter('startup_wait_sec', 5.0)
        self.declare_parameter('aging_weight', 0.08)
        self.declare_parameter('deadhead_weight', 0.25)
        self.declare_parameter('retry_limit', 1)
        self.declare_parameter('seed', 0)
        self.declare_parameter('job_sequence', [])

        robot_names = list(self.get_parameter('robots').value)
        self.job_interval_sec = float(self.get_parameter('job_interval_sec').value)
        self.max_jobs = int(self.get_parameter('max_jobs').value)
        self.pickup_wait_sec = float(self.get_parameter('pickup_wait_sec').value)
        self.assignment_stagger_sec = float(self.get_parameter('assignment_stagger_sec').value)
        self.startup_wait_sec = float(self.get_parameter('startup_wait_sec').value)
        self.aging_weight = float(self.get_parameter('aging_weight').value)
        self.deadhead_weight = float(self.get_parameter('deadhead_weight').value)
        self.retry_limit = int(self.get_parameter('retry_limit').value)

        seed = int(self.get_parameter('seed').value)
        self.random = random.Random(seed if seed != 0 else None)
        self.job_sequence = self.parse_job_sequence(list(self.get_parameter('job_sequence').value))

        self.robots: List[RobotWorker] = [
            RobotWorker(name, ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose'))
            for name in robot_names
        ]

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.initial_pose_publishers = {
            name: self.create_publisher(PoseWithCovarianceStamped, f'/{name}/initialpose', qos)
            for name in robot_names
            if name in INITIAL_POSES
        }

        self.job_queue: List[DeliveryJob] = []
        self.next_job_id = 1
        self.next_sequence_index = 0
        self.generated_jobs = 0
        self.completed_jobs = 0
        self.failed_jobs = 0
        self.retries = 0

        self.generator_timer = None
        self.dispatch_timer = None
        self.last_assignment_time = None

    def run(self) -> bool:
        if not self.wait_for_nav2():
            return False

        self.get_logger().info(
            f'Dispatcher ready for {self.robot_names()}. '
            f'Policy: shortest job first + aging. max_jobs={self.max_jobs}'
        )
        self.sleep_with_spin(self.startup_wait_sec)

        self.generator_timer = self.create_timer(self.job_interval_sec, self.generate_job)
        self.dispatch_timer = self.create_timer(0.5, self.dispatch_jobs)
        self.generate_job()

        while rclpy.ok() and not self.is_done():
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info(
            f'Done. generated={self.generated_jobs}, completed={self.completed_jobs}, '
            f'failed={self.failed_jobs}, retries={self.retries}'
        )
        return self.failed_jobs == 0

    def wait_for_nav2(self) -> bool:
        for robot in self.robots:
            topic = f'/{robot.name}/navigate_to_pose'
            self.get_logger().info(f'Waiting for {topic}...')
            if not robot.action_client.wait_for_server(timeout_sec=35.0):
                self.get_logger().error(f'{topic} is not available')
                return False

        self.publish_initial_poses()
        return True

    def publish_initial_poses(self, count: int = 6):
        for attempt in range(1, count + 1):
            for robot_name, publisher in self.initial_pose_publishers.items():
                pose = INITIAL_POSES[robot_name]
                publisher.publish(self.make_initial_pose(*pose))
                x, y, yaw = pose
                self.get_logger().info(
                    f'Initial pose {attempt}/{count} for {robot_name}: '
                    f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
                )
            self.sleep_with_spin(0.5)

    def make_initial_pose(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
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

    def robot_names(self) -> str:
        return ', '.join(robot.name for robot in self.robots)

    def is_done(self) -> bool:
        generated_all_jobs = self.generated_jobs >= self.max_jobs
        robots_are_idle = all(not robot.busy for robot in self.robots)
        return generated_all_jobs and not self.job_queue and robots_are_idle

    def parse_job_sequence(self, items) -> List[Tuple[str, str]]:
        jobs = []
        for item in items:
            pickup, dropoff = [part.strip().upper() for part in item.split(':', 1)]
            if pickup in WAYPOINTS and dropoff in WAYPOINTS and pickup != dropoff:
                jobs.append((pickup, dropoff))
        return jobs

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def generate_job(self):
        if self.generated_jobs >= self.max_jobs:
            if self.generator_timer:
                self.generator_timer.cancel()
            return

        if self.job_sequence:
            if self.next_sequence_index >= len(self.job_sequence):
                if self.generator_timer:
                    self.generator_timer.cancel()
                return
            pickup, dropoff = self.job_sequence[self.next_sequence_index]
            self.next_sequence_index += 1
        else:
            pickup, dropoff = self.random.sample(list(WAYPOINTS.keys()), 2)

        job = DeliveryJob(self.next_job_id, pickup, dropoff, self.now_sec())
        self.next_job_id += 1
        self.generated_jobs += 1
        self.job_queue.append(job)
        self.get_logger().info(f'Queued job {job.job_id}: {job.route}')

    def dispatch_jobs(self):
        for robot in self.available_robots():
            if not self.job_queue or not self.assignment_ready():
                return

            job, score = self.choose_job(robot)
            self.job_queue.remove(job)
            self.last_assignment_time = self.get_clock().now()
            robot.busy = True
            robot.job = job
            robot.phase = 'pickup'
            job.attempts += 1

            self.get_logger().info(
                f'Assigned job {job.job_id} ({job.route}) to {robot.name}, score={score:.2f}'
            )
            self.send_robot_to_station(robot, job.pickup)

    def available_robots(self) -> List[RobotWorker]:
        return [robot for robot in self.robots if not robot.busy]

    def assignment_ready(self) -> bool:
        if self.last_assignment_time is None:
            return True
        elapsed = self.get_clock().now() - self.last_assignment_time
        return elapsed.nanoseconds >= int(self.assignment_stagger_sec * 1_000_000_000)

    def choose_job(self, robot: RobotWorker) -> Tuple[DeliveryJob, float]:
        scored_jobs = [(self.score_job(job, robot), job) for job in self.job_queue]
        scored_jobs.sort(key=lambda item: (item[0], item[1].created_time, item[1].job_id))
        return scored_jobs[0][1], scored_jobs[0][0]

    def score_job(self, job: DeliveryJob, robot: RobotWorker) -> float:
        job_length = station_distance(job.pickup, job.dropoff)
        pickup_distance = self.distance_to_pickup(robot, job)
        waiting_time = max(0.0, self.now_sec() - job.created_time)
        return job_length + self.deadhead_weight * pickup_distance - self.aging_weight * waiting_time

    def distance_to_pickup(self, robot: RobotWorker, job: DeliveryJob) -> float:
        if robot.last_station in WAYPOINTS:
            return station_distance(robot.last_station, job.pickup)
        if robot.name in INITIAL_POSES:
            return pose_to_station_distance(INITIAL_POSES[robot.name], job.pickup)
        return 0.0

    def send_robot_to_station(self, robot: RobotWorker, station: str):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(*WAYPOINTS[station])
        self.get_logger().info(f'{robot.name} job {robot.job.job_id}: going to {station}')

        future = robot.action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda done, r=robot, s=station: self.goal_response(done, r, s))

    def goal_response(self, future, robot: RobotWorker, station: str):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.fail_job(robot, f'goal to {station} was rejected')
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
            self.fail_job(robot, f'goal to {station} failed with status {status}')
            return

        job = robot.job
        robot.last_station = station
        if robot.phase == 'pickup':
            self.get_logger().info(f'{robot.name} picked up job {job.job_id} at {station}')
            robot.phase = 'dropoff_wait'
            wait_timer = None

            def done_waiting(r=robot):
                wait_timer.cancel()
                self.start_dropoff(r)

            wait_timer = self.create_timer(self.pickup_wait_sec, done_waiting)
            return

        if robot.phase == 'dropoff':
            self.completed_jobs += 1
            self.get_logger().info(f'{robot.name} completed job {job.job_id}: {job.route}')
            self.release_robot(robot)

    def start_dropoff(self, robot: RobotWorker):
        if robot.phase != 'dropoff_wait':
            return
        robot.phase = 'dropoff'
        self.send_robot_to_station(robot, robot.job.dropoff)

    def fail_job(self, robot: RobotWorker, reason: str):
        job = robot.job
        if job is None:
            self.get_logger().error(f'{robot.name} failed: {reason}')
            self.release_robot(robot)
            return

        if job.attempts <= self.retry_limit:
            self.retries += 1
            self.get_logger().warn(f'Job {job.job_id} failed, trying again later: {reason}')
            self.release_robot(robot)
            self.job_queue.append(job)
            return

        self.failed_jobs += 1
        self.get_logger().error(f'Job {job.job_id} failed permanently: {reason}')
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
