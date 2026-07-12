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
    created_time_sec: float
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
    completed_jobs: int = 0
    failed_attempts: int = 0
    distance_traveled_estimate: float = 0.0


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def station_distance(a: str, b: str) -> float:
    ax, ay, _ = WAYPOINTS[a]
    bx, by, _ = WAYPOINTS[b]
    return math.hypot(ax - bx, ay - by)


def pose_distance_to_station(pose: StationPose, station: str) -> float:
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
        self.declare_parameter('retry_delay_sec', 3.0)
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
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        seed = int(self.get_parameter('seed').value)
        self.random = random.Random(seed if seed != 0 else None)
        self.job_sequence = self.parse_job_sequence(
            list(self.get_parameter('job_sequence').value)
        )

        self.robots: List[RobotWorker] = [
            RobotWorker(
                name=name,
                action_client=ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose'),
                last_station=None,
            )
            for name in robot_names
        ]
        initial_pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.initial_pose_publishers = {
            name: self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{name}/initialpose',
                initial_pose_qos,
            )
            for name in robot_names
            if name in INITIAL_POSES
        }
        self.job_queue: List[DeliveryJob] = []
        self.next_job_id = 1
        self.next_sequence_index = 0
        self.generated_jobs = 0
        self.completed_jobs = 0
        self.permanently_failed_jobs = 0
        self.retry_attempts = 0
        self.pending_retries = 0

        self.generator_timer = None
        self.dispatch_timer = None
        self.status_timer = None
        self.last_assignment_time = None

    def run(self) -> bool:
        if not self.wait_for_robot_servers():
            return False

        self.get_logger().info(
            f'Scheduler dispatcher ready: robots={self.robot_names()}, '
            f'policy=SJF+aging, job_interval={self.job_interval_sec:.1f}s, '
            f'max_jobs={self.max_jobs}, aging_weight={self.aging_weight:.3f}, '
            f'deadhead_weight={self.deadhead_weight:.3f}, retry_limit={self.retry_limit}, '
            f'job_sequence={self.format_job_sequence()}. '
            f'Waiting {self.startup_wait_sec:.1f}s for AMCL poses to settle.'
        )
        self.sleep_with_spin(self.startup_wait_sec)
        self.generator_timer = self.create_timer(self.job_interval_sec, self.generate_job_timer)
        self.dispatch_timer = self.create_timer(0.5, self.dispatch_jobs)
        self.status_timer = self.create_timer(5.0, self.log_scheduler_snapshot)

        self.generate_job_timer()

        while rclpy.ok() and not self.is_done():
            rclpy.spin_once(self, timeout_sec=0.2)

        self.log_scheduler_snapshot()
        self.get_logger().info(
            f'Dispatcher finished. Generated={self.generated_jobs}, '
            f'Completed={self.completed_jobs}, PermanentFailures={self.permanently_failed_jobs}, '
            f'Retries={self.retry_attempts}'
        )
        return self.permanently_failed_jobs == 0

    def wait_for_robot_servers(self) -> bool:
        for robot in self.robots:
            self.get_logger().info(f'Waiting for /{robot.name}/navigate_to_pose action server...')
            if not robot.action_client.wait_for_server(timeout_sec=35.0):
                self.get_logger().error(f'/{robot.name}/navigate_to_pose is not available.')
                return False

        self.publish_initial_poses_for_amcl()
        self.get_logger().info(
            'Dispatcher will let each namespaced Nav2 stack validate localization '
            'when goals are submitted.'
        )
        return True

    def publish_initial_poses_for_amcl(self, count: int = 6):
        if not self.initial_pose_publishers:
            return

        self.get_logger().info('Publishing initial poses from dispatcher to wake AMCL...')
        for attempt in range(1, count + 1):
            for robot_name, publisher in self.initial_pose_publishers.items():
                pose = INITIAL_POSES[robot_name]
                publisher.publish(self.make_initial_pose(*pose))
                x, y, yaw = pose
                self.get_logger().info(
                    f'Published dispatcher initial pose {attempt}/{count} for '
                    f'{robot_name}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
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
        all_jobs_generated = self.generated_jobs >= self.max_jobs
        all_robots_idle = all(not robot.busy for robot in self.robots)
        return all_jobs_generated and not self.job_queue and all_robots_idle and self.pending_retries == 0

    def parse_job_sequence(self, sequence_param) -> List[Tuple[str, str]]:
        sequence = []
        for item in sequence_param:
            if ':' not in item:
                raise ValueError(f'Invalid job_sequence item {item!r}; expected PICKUP:DROPOFF')
            pickup, dropoff = [part.strip().upper() for part in item.split(':', 1)]
            if pickup not in WAYPOINTS or dropoff not in WAYPOINTS:
                valid = ', '.join(sorted(WAYPOINTS))
                raise ValueError(f'Invalid job_sequence item {item!r}; valid stations: {valid}')
            if pickup == dropoff:
                raise ValueError(f'Invalid job_sequence item {item!r}; pickup and dropoff must differ')
            sequence.append((pickup, dropoff))
        return sequence

    def format_job_sequence(self) -> str:
        if not self.job_sequence:
            return 'random'
        return ', '.join(f'{pickup}:{dropoff}' for pickup, dropoff in self.job_sequence)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def generate_job_timer(self):
        if self.generated_jobs >= self.max_jobs:
            if self.generator_timer is not None:
                self.generator_timer.cancel()
            return

        if self.job_sequence:
            if self.next_sequence_index >= len(self.job_sequence):
                if self.generator_timer is not None:
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

        self.get_logger().info(
            f'Queued job {job.job_id}: {job.route}; '
            f'base_distance={self.job_service_distance(job):.2f}, queue_size={len(self.job_queue)}'
        )

    def dispatch_jobs(self):
        for robot in self.available_robots():
            if not self.job_queue:
                return
            if not self.assignment_window_ready():
                return

            job, score = self.choose_job_for_robot(robot)
            self.job_queue.remove(job)
            self.last_assignment_time = self.get_clock().now()
            robot.busy = True
            robot.job = job
            robot.phase = 'pickup'
            job.attempts += 1
            self.get_logger().info(
                f'Scheduler assigned job {job.job_id} ({job.route}) to {robot.name}; '
                f'effective_score={score:.2f}, attempt={job.attempts}/{self.retry_limit + 1}, '
                f'queue_size={len(self.job_queue)}'
            )
            self.send_robot_to_station(robot, job.pickup)

    def available_robots(self) -> List[RobotWorker]:
        return [robot for robot in self.robots if not robot.busy]

    def assignment_window_ready(self) -> bool:
        if self.last_assignment_time is None:
            return True
        elapsed = self.get_clock().now() - self.last_assignment_time
        return elapsed.nanoseconds >= int(self.assignment_stagger_sec * 1_000_000_000)

    def choose_job_for_robot(self, robot: RobotWorker) -> Tuple[DeliveryJob, float]:
        scored = [(self.effective_job_score(job, robot), job) for job in self.job_queue]
        scored.sort(key=lambda item: (item[0], item[1].created_time_sec, item[1].job_id))
        return scored[0][1], scored[0][0]

    def effective_job_score(self, job: DeliveryJob, robot: RobotWorker) -> float:
        service = self.job_service_distance(job)
        deadhead = self.robot_to_pickup_distance(robot, job)
        wait_age = max(0.0, self.now_sec() - job.created_time_sec)
        retry_penalty = 0.35 * job.attempts
        return service + self.deadhead_weight * deadhead + retry_penalty - self.aging_weight * wait_age

    def job_service_distance(self, job: DeliveryJob) -> float:
        return station_distance(job.pickup, job.dropoff)

    def robot_to_pickup_distance(self, robot: RobotWorker, job: DeliveryJob) -> float:
        if robot.last_station in WAYPOINTS:
            return station_distance(robot.last_station, job.pickup)
        if robot.name in INITIAL_POSES:
            return pose_distance_to_station(INITIAL_POSES[robot.name], job.pickup)
        return 0.0

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
        robot.last_station = station
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
            robot.completed_jobs += 1
            robot.distance_traveled_estimate += self.job_service_distance(job)
            self.get_logger().info(
                f'{robot.name} job {job.job_id} complete: {job.route}. '
                f'Completed={self.completed_jobs}, PermanentFailures={self.permanently_failed_jobs}'
            )
            self.release_robot(robot)

    def start_dropoff_once(self, robot: RobotWorker):
        if robot.phase != 'dropoff_wait':
            return
        robot.phase = 'dropoff'
        self.send_robot_to_station(robot, robot.job.dropoff)

    def fail_robot_job(self, robot: RobotWorker, reason: str):
        job = robot.job
        robot.failed_attempts += 1
        if job is None:
            self.get_logger().error(f'{robot.name}: failed without an assigned job: {reason}')
            self.release_robot(robot)
            return

        if job.attempts <= self.retry_limit:
            self.retry_attempts += 1
            self.pending_retries += 1
            self.get_logger().warn(
                f'{robot.name} job {job.job_id} failed attempt {job.attempts}: {reason}. '
                f'Requeueing after {self.retry_delay_sec:.1f}s with aging preserved.'
            )
            self.release_robot(robot)
            retry_timer = None

            def requeue_job(j=job):
                retry_timer.cancel()
                self.pending_retries = max(0, self.pending_retries - 1)
                self.job_queue.append(j)
                self.get_logger().info(
                    f'Requeued job {j.job_id}: {j.route}; attempts={j.attempts}, '
                    f'queue_size={len(self.job_queue)}'
                )

            retry_timer = self.create_timer(self.retry_delay_sec, requeue_job)
            return

        self.permanently_failed_jobs += 1
        self.get_logger().error(
            f'{robot.name} job {job.job_id} permanently failed: {job.route}; {reason}. '
            f'Completed={self.completed_jobs}, PermanentFailures={self.permanently_failed_jobs}'
        )
        self.release_robot(robot)

    def release_robot(self, robot: RobotWorker):
        robot.busy = False
        robot.job = None
        robot.phase = 'idle'
        robot.goal_handle = None

    def log_scheduler_snapshot(self):
        robot_state = ', '.join(
            f'{robot.name}:{robot.phase}'
            + (f':job{robot.job.job_id}' if robot.job else '')
            for robot in self.robots
        )
        queue_state = self.describe_queue()
        self.get_logger().info(
            f'Scheduler snapshot | queue=[{queue_state}] | robots=[{robot_state}] | '
            f'completed={self.completed_jobs}, failed={self.permanently_failed_jobs}, '
            f'retries={self.retry_attempts}, pending_retries={self.pending_retries}'
        )

    def describe_queue(self) -> str:
        if not self.job_queue:
            return 'empty'
        sample = []
        idle_robot = next((robot for robot in self.robots if not robot.busy), self.robots[0])
        for job in sorted(self.job_queue, key=lambda j: self.effective_job_score(j, idle_robot))[:5]:
            age = self.now_sec() - job.created_time_sec
            score = self.effective_job_score(job, idle_robot)
            sample.append(f'job{job.job_id}:{job.route}:score={score:.2f}:age={age:.1f}s')
        return '; '.join(sample)

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
