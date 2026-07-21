import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FleetMetricsNode(Node):
    """Collects live job and robot movement metrics from ROS topics."""

    def __init__(self):
        super().__init__('fleet_metrics_node')
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('results_dir', '')
        self.declare_parameter('publish_period_sec', 1.0)

        self.robot_names = list(self.get_parameter('robots').value)
        results_dir = str(self.get_parameter('results_dir').value).strip()
        self.output_path = Path(results_dir) / 'fleet_metrics.json' if results_dir else None

        self.jobs = {}
        self.event_counts = {}
        self.robot_distance = {name: 0.0 for name in self.robot_names}
        self.last_positions = {}

        self.create_subscription(String, '/warehouse/events', self.event_callback, 50)
        for robot_name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_name}/amcl_pose',
                lambda message, name=robot_name: self.pose_callback(name, message),
                10,
            )

        self.metrics_publisher = self.create_publisher(
            String, '/warehouse/fleet_metrics', 10
        )
        period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(period, self.publish_metrics)
        self.get_logger().info(
            f'Collecting metrics for {", ".join(self.robot_names)}'
        )

    def event_callback(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignored an invalid warehouse event')
            return

        event_type = event.get('event', 'unknown')
        self.event_counts[event_type] = self.event_counts.get(event_type, 0) + 1

        job_id = event.get('job_id')
        if job_id is None:
            return

        job = self.jobs.setdefault(str(job_id), {'status': 'queued'})
        if event_type == 'job_queued':
            job['status'] = 'queued'
        elif event_type == 'job_assigned':
            job['status'] = 'running'
            job['robot'] = event.get('robot', '')
        elif event_type == 'job_retry':
            job['status'] = 'queued'
        elif event_type == 'job_completed':
            job['status'] = 'completed'
        elif event_type == 'job_failed':
            job['status'] = 'failed'

    def pose_callback(self, robot_name, message):
        position = message.pose.pose.position
        current = (position.x, position.y)
        previous = self.last_positions.get(robot_name)
        self.last_positions[robot_name] = current
        if previous is None:
            return

        distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        # Ignore localization jumps so they do not count as robot travel.
        if distance < 1.0:
            self.robot_distance[robot_name] += distance

    def make_metrics(self):
        statuses = [job['status'] for job in self.jobs.values()]
        return {
            'jobs_seen': len(self.jobs),
            'jobs_queued': statuses.count('queued'),
            'jobs_running': statuses.count('running'),
            'jobs_completed': statuses.count('completed'),
            'jobs_failed': statuses.count('failed'),
            'robot_distance_m': {
                name: round(distance, 3)
                for name, distance in self.robot_distance.items()
            },
            'event_counts': self.event_counts,
        }

    def publish_metrics(self):
        metrics = self.make_metrics()
        message = String()
        message.data = json.dumps(metrics)
        self.metrics_publisher.publish(message)

        if self.output_path:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text(json.dumps(metrics, indent=2), encoding='utf-8')


def main():
    rclpy.init()
    node = FleetMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_metrics()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
