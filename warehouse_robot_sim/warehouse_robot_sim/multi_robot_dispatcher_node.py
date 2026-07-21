from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
from statistics import mean
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from warehouse_robot_sim.layout_config import initial_poses, load_layout, station_poses

StationPose = Tuple[float, float, float]
POLICIES = {'fcfs', 'sjf', 'sjf_aging', 'nearest_robot'}


@dataclass
class DeliveryJob:
    job_id: int
    pickup: str
    dropoff: str
    created_time: float
    attempts: int = 0
    assigned_time: Optional[float] = None
    pickup_time: Optional[float] = None
    completed_time: Optional[float] = None
    robot_name: str = ''
    status: str = 'queued'
    failure_reason: str = ''

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
    busy_started: Optional[float] = None
    total_busy_sec: float = 0.0


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class MultiRobotDispatcherNode(Node):
    def __init__(self):
        super().__init__('multi_robot_dispatcher_node')
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('layout', 'standard')
        self.declare_parameter('scheduler_policy', 'sjf_aging')
        self.declare_parameter('job_interval_sec', 4.0)
        self.declare_parameter('arrival_jitter_sec', 1.5)
        self.declare_parameter('max_jobs', 9)
        self.declare_parameter('pickup_wait_sec', 1.5)
        self.declare_parameter('assignment_stagger_sec', 1.0)
        self.declare_parameter('startup_wait_sec', 5.0)
        self.declare_parameter('aging_weight', 0.08)
        self.declare_parameter('deadhead_weight', 0.25)
        self.declare_parameter('retry_limit', 1)
        self.declare_parameter('seed', 1)
        self.declare_parameter('job_sequence', [''])
        self.declare_parameter('results_dir', '')

        layout_name = str(self.get_parameter('layout').value)
        layout = load_layout(layout_name)
        self.layout_name = layout_name
        self.initial_poses: Dict[str, StationPose] = initial_poses(layout)
        self.waypoints: Dict[str, StationPose] = station_poses(layout)

        robot_names = list(self.get_parameter('robots').value)
        unknown_robots = set(robot_names).difference(self.initial_poses)
        if unknown_robots:
            raise ValueError(f'Unknown robots: {", ".join(sorted(unknown_robots))}')

        self.scheduler_policy = str(self.get_parameter('scheduler_policy').value)
        if self.scheduler_policy not in POLICIES:
            raise ValueError(f'Unknown scheduler policy: {self.scheduler_policy}')

        self.job_interval_sec = float(self.get_parameter('job_interval_sec').value)
        self.arrival_jitter_sec = float(self.get_parameter('arrival_jitter_sec').value)
        self.max_jobs = int(self.get_parameter('max_jobs').value)
        self.pickup_wait_sec = float(self.get_parameter('pickup_wait_sec').value)
        self.assignment_stagger_sec = float(
            self.get_parameter('assignment_stagger_sec').value
        )
        self.startup_wait_sec = float(self.get_parameter('startup_wait_sec').value)
        self.aging_weight = float(self.get_parameter('aging_weight').value)
        self.deadhead_weight = float(self.get_parameter('deadhead_weight').value)
        self.retry_limit = int(self.get_parameter('retry_limit').value)
        self.seed = int(self.get_parameter('seed').value)
        self.random = random.Random(self.seed)
        self.job_sequence = self.parse_job_sequence(
            list(self.get_parameter('job_sequence').value)
        )

        results_value = str(self.get_parameter('results_dir').value).strip()
        self.results_dir = Path(results_value) if results_value else None
        self.events_path = self.results_dir / 'events.jsonl' if self.results_dir else None
        if self.results_dir:
            self.results_dir.mkdir(parents=True, exist_ok=True)
            self.events_path.write_text('', encoding='utf-8')

        self.event_publisher = self.create_publisher(String, '/warehouse/events', 50)

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
            name: self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{name}/initialpose',
                qos,
            )
            for name in robot_names
        }

        self.job_queue: List[DeliveryJob] = []
        self.jobs: List[DeliveryJob] = []
        self.next_job_id = 1
        self.next_sequence_index = 0
        self.generated_jobs = 0
        self.completed_jobs = 0
        self.failed_jobs = 0
        self.retries = 0
        self.generation_finished = False
        self.results_written = False
        self.experiment_start_time: Optional[float] = None

        self.generator_timer = None
        self.dispatch_timer = None
        self.last_assignment_time = None

    def run(self) -> bool:
        if not self.wait_for_nav2():
            self.write_results('startup_failed')
            return False

        self.get_logger().info(
            f'Dispatcher ready for {self.robot_names()}. '
            f'Policy={self.scheduler_policy}, layout={self.layout_name}, '
            f'max_jobs={self.max_jobs}, seed={self.seed}'
        )
        self.sleep_with_spin(self.startup_wait_sec)
        self.experiment_start_time = self.now_sec()
        self.record_event('experiment_started')

        self.dispatch_timer = self.create_timer(0.5, self.dispatch_jobs)
        self.generate_job()

        while rclpy.ok() and not self.is_done():
            rclpy.spin_once(self, timeout_sec=0.2)

        status = 'completed' if self.failed_jobs == 0 else 'completed_with_failures'
        self.write_results(status)
        self.get_logger().info(
            f'Done. generated={self.generated_jobs}, completed={self.completed_jobs}, '
            f'failed={self.failed_jobs}, retries={self.retries}'
        )
        return self.failed_jobs == 0

    def wait_for_nav2(self) -> bool:
        for robot in self.robots:
            topic = f'/{robot.name}/navigate_to_pose'
            self.get_logger().info(f'Waiting for {topic}...')
            if not robot.action_client.wait_for_server(timeout_sec=45.0):
                self.get_logger().error(f'{topic} is not available')
                return False

        self.publish_initial_poses()
        return True

    def publish_initial_poses(self, count: int = 6):
        for attempt in range(1, count + 1):
            for robot_name, publisher in self.initial_pose_publishers.items():
                pose = self.initial_poses[robot_name]
                publisher.publish(self.make_initial_pose(*pose))
                x, y, yaw = pose
                self.get_logger().info(
                    f'Initial pose {attempt}/{count} for {robot_name}: '
                    f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
                )
            self.sleep_with_spin(0.5)

    def make_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
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
        robots_are_idle = all(not robot.busy for robot in self.robots)
        return self.generation_finished and not self.job_queue and robots_are_idle

    def parse_job_sequence(self, items) -> List[Tuple[str, str]]:
        jobs = []
        for item in items:
            if ':' not in item:
                continue
            pickup, dropoff = [part.strip().upper() for part in item.split(':', 1)]
            if pickup in self.waypoints and dropoff in self.waypoints and pickup != dropoff:
                jobs.append((pickup, dropoff))
        return jobs

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def has_more_jobs(self) -> bool:
        if self.generated_jobs >= self.max_jobs:
            return False
        if self.job_sequence and self.next_sequence_index >= len(self.job_sequence):
            return False
        return True

    def generate_job(self):
        if self.generator_timer:
            self.generator_timer.cancel()
            self.generator_timer = None

        if not self.has_more_jobs():
            self.generation_finished = True
            self.record_event('generation_finished')
            return

        if self.job_sequence:
            pickup, dropoff = self.job_sequence[self.next_sequence_index]
            self.next_sequence_index += 1
        else:
            # The seed makes randomized experiments repeatable.
            pickup, dropoff = self.random.sample(list(self.waypoints), 2)

        job = DeliveryJob(self.next_job_id, pickup, dropoff, self.now_sec())
        self.next_job_id += 1
        self.generated_jobs += 1
        self.job_queue.append(job)
        self.jobs.append(job)
        self.get_logger().info(f'Queued job {job.job_id}: {job.route}')
        self.record_event('job_queued', job=job)

        if self.has_more_jobs():
            low = max(0.2, self.job_interval_sec - self.arrival_jitter_sec)
            high = max(low, self.job_interval_sec + self.arrival_jitter_sec)
            delay = self.random.uniform(low, high)
            self.generator_timer = self.create_timer(delay, self.generate_job)
            self.record_event('next_arrival_scheduled', delay_sec=round(delay, 3))
        else:
            self.generation_finished = True
            self.record_event('generation_finished')

    def dispatch_jobs(self):
        if not self.job_queue or not self.assignment_ready():
            return

        available = self.available_robots()
        if not available:
            return

        robot, job, score = self.choose_assignment(available)
        self.job_queue.remove(job)
        self.last_assignment_time = self.get_clock().now()
        robot.busy = True
        robot.job = job
        robot.phase = 'pickup'
        robot.busy_started = self.now_sec()
        job.attempts += 1
        job.robot_name = robot.name
        job.status = 'running'
        if job.assigned_time is None:
            job.assigned_time = self.now_sec()

        self.get_logger().info(
            f'Assigned job {job.job_id} ({job.route}) to {robot.name}, score={score:.2f}'
        )
        self.record_event('job_assigned', job=job, robot=robot.name, score=round(score, 3))
        self.send_robot_to_station(robot, job.pickup)

    def available_robots(self) -> List[RobotWorker]:
        return [robot for robot in self.robots if not robot.busy]

    def assignment_ready(self) -> bool:
        if self.last_assignment_time is None:
            return True
        elapsed = self.get_clock().now() - self.last_assignment_time
        return elapsed.nanoseconds >= int(self.assignment_stagger_sec * 1_000_000_000)

    def choose_assignment(self, robots):
        # Score every available robot/job pair and take the lowest score.
        choices = []
        for robot in robots:
            for job in self.job_queue:
                score = self.score_job(job, robot)
                distance = self.distance_to_pickup(robot, job)
                choices.append((score, job.created_time, distance, robot.name, robot, job))
        choices.sort(key=lambda item: item[:4])
        score, _, _, _, robot, job = choices[0]
        return robot, job, score

    def score_job(self, job: DeliveryJob, robot: RobotWorker) -> float:
        job_length = self.station_distance(job.pickup, job.dropoff)
        pickup_distance = self.distance_to_pickup(robot, job)
        waiting_time = max(0.0, self.now_sec() - job.created_time)

        if self.scheduler_policy == 'fcfs':
            return 0.0
        if self.scheduler_policy == 'sjf':
            return job_length
        if self.scheduler_policy == 'nearest_robot':
            return pickup_distance

        # Aging lowers the score as a job waits, which reduces starvation.
        return (
            job_length
            + self.deadhead_weight * pickup_distance
            - self.aging_weight * waiting_time
        )

    def station_distance(self, a: str, b: str) -> float:
        ax, ay, _ = self.waypoints[a]
        bx, by, _ = self.waypoints[b]
        return math.hypot(ax - bx, ay - by)

    def distance_to_pickup(self, robot: RobotWorker, job: DeliveryJob) -> float:
        if robot.last_station in self.waypoints:
            return self.station_distance(robot.last_station, job.pickup)
        px, py, _ = self.initial_poses[robot.name]
        sx, sy, _ = self.waypoints[job.pickup]
        return math.hypot(px - sx, py - sy)

    def send_robot_to_station(self, robot: RobotWorker, station: str):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(*self.waypoints[station])
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
            job.pickup_time = self.now_sec()
            self.get_logger().info(f'{robot.name} picked up job {job.job_id} at {station}')
            self.record_event('pickup_complete', job=job, robot=robot.name)
            robot.phase = 'dropoff_wait'
            wait_timer = None

            def done_waiting(r=robot):
                wait_timer.cancel()
                self.start_dropoff(r)

            wait_timer = self.create_timer(self.pickup_wait_sec, done_waiting)
            return

        if robot.phase == 'dropoff':
            job.completed_time = self.now_sec()
            job.status = 'completed'
            self.completed_jobs += 1
            self.get_logger().info(f'{robot.name} completed job {job.job_id}: {job.route}')
            self.record_event('job_completed', job=job, robot=robot.name)
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
            job.status = 'queued'
            self.get_logger().warning(f'Job {job.job_id} failed, trying again later: {reason}')
            self.record_event('job_retry', job=job, robot=robot.name, reason=reason)
            self.release_robot(robot)
            self.job_queue.append(job)
            return

        job.status = 'failed'
        job.failure_reason = reason
        job.completed_time = self.now_sec()
        self.failed_jobs += 1
        self.get_logger().error(f'Job {job.job_id} failed permanently: {reason}')
        self.record_event('job_failed', job=job, robot=robot.name, reason=reason)
        self.release_robot(robot)

    def release_robot(self, robot: RobotWorker):
        if robot.busy_started is not None:
            robot.total_busy_sec += max(0.0, self.now_sec() - robot.busy_started)
        robot.busy = False
        robot.busy_started = None
        robot.job = None
        robot.phase = 'idle'
        robot.goal_handle = None

    def record_event(self, event_type: str, job=None, **details):
        event = {'event': event_type, 'sim_time_sec': round(self.now_sec(), 3)}
        if job:
            event.update({'job_id': job.job_id, 'route': job.route})
        event.update(details)
        event_json = json.dumps(event)

        message = String()
        message.data = event_json
        self.event_publisher.publish(message)

        if self.events_path:
            with self.events_path.open('a', encoding='utf-8') as event_file:
                event_file.write(event_json + '\n')

    def write_results(self, status: str):
        if self.results_written or not self.results_dir:
            return

        end_time = self.now_sec()
        start_time = self.experiment_start_time or end_time
        makespan = max(0.0, end_time - start_time)
        wait_times = [
            job.assigned_time - job.created_time
            for job in self.jobs
            if job.assigned_time is not None
        ]
        completion_times = [
            job.completed_time - job.created_time
            for job in self.jobs
            if job.status == 'completed' and job.completed_time is not None
        ]
        utilization = {
            robot.name: round(robot.total_busy_sec / makespan, 4) if makespan else 0.0
            for robot in self.robots
        }
        failures = Counter(job.failure_reason for job in self.jobs if job.failure_reason)

        # summary.json is used by the dashboard; jobs.csv keeps per-job details.
        summary = {
            'status': status,
            'layout': self.layout_name,
            'scheduler_policy': self.scheduler_policy,
            'seed': self.seed,
            'robot_count': len(self.robots),
            'jobs_generated': self.generated_jobs,
            'jobs_completed': self.completed_jobs,
            'jobs_failed': self.failed_jobs,
            'retries': self.retries,
            'average_wait_sec': round(mean(wait_times), 3) if wait_times else 0.0,
            'average_completion_sec': (
                round(mean(completion_times), 3) if completion_times else 0.0
            ),
            'makespan_sec': round(makespan, 3),
            'throughput_jobs_per_min': (
                round(self.completed_jobs * 60.0 / makespan, 3) if makespan else 0.0
            ),
            'robot_utilization': utilization,
            'failure_reasons': dict(failures),
        }
        summary_path = self.results_dir / 'summary.json'
        summary_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        self.write_jobs_csv()
        self.record_event('experiment_finished', status=status)
        self.results_written = True

    def write_jobs_csv(self):
        columns = [
            'job_id', 'pickup', 'dropoff', 'robot', 'status', 'attempts',
            'created_time', 'assigned_time', 'pickup_time', 'completed_time',
            'wait_sec', 'completion_sec', 'failure_reason',
        ]
        with (self.results_dir / 'jobs.csv').open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=columns)
            writer.writeheader()
            for job in self.jobs:
                wait = job.assigned_time - job.created_time if job.assigned_time else ''
                completion = (
                    job.completed_time - job.created_time if job.completed_time else ''
                )
                writer.writerow({
                    'job_id': job.job_id,
                    'pickup': job.pickup,
                    'dropoff': job.dropoff,
                    'robot': job.robot_name,
                    'status': job.status,
                    'attempts': job.attempts,
                    'created_time': round(job.created_time, 3),
                    'assigned_time': self.round_optional(job.assigned_time),
                    'pickup_time': self.round_optional(job.pickup_time),
                    'completed_time': self.round_optional(job.completed_time),
                    'wait_sec': round(wait, 3) if wait != '' else '',
                    'completion_sec': round(completion, 3) if completion != '' else '',
                    'failure_reason': job.failure_reason,
                })

    @staticmethod
    def round_optional(value):
        return round(value, 3) if value is not None else ''

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
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


def main():
    rclpy.init()
    node = MultiRobotDispatcherNode()
    success = False
    try:
        success = node.run()
    except KeyboardInterrupt:
        node.write_results('cancelled')
        success = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
