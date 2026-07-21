import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from warehouse_robot_sim.layout_config import load_layout, station_poses


class StationOccupancyNode(Node):
    """Tracks which robots are currently near each warehouse station."""

    def __init__(self):
        super().__init__('station_occupancy_node')
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('layout', 'standard')
        self.declare_parameter('results_dir', '')
        self.declare_parameter('station_radius_m', 0.9)

        self.robot_names = list(self.get_parameter('robots').value)
        layout_name = str(self.get_parameter('layout').value)
        self.stations = station_poses(load_layout(layout_name))
        self.station_radius = float(self.get_parameter('station_radius_m').value)
        results_dir = str(self.get_parameter('results_dir').value).strip()
        self.output_path = (
            Path(results_dir) / 'station_occupancy.json' if results_dir else None
        )

        self.robot_stations = {name: None for name in self.robot_names}
        self.station_visits = {name: 0 for name in self.stations}
        self.max_occupancy = {name: 0 for name in self.stations}

        for robot_name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_name}/amcl_pose',
                lambda message, name=robot_name: self.pose_callback(name, message),
                10,
            )

        self.occupancy_publisher = self.create_publisher(
            String, '/warehouse/station_occupancy', 10
        )
        self.create_timer(1.0, self.publish_occupancy)
        self.get_logger().info(
            f'Tracking station occupancy for {", ".join(self.robot_names)} '
            f'on {layout_name}'
        )

    def pose_callback(self, robot_name, message):
        position = message.pose.pose.position
        station = self.nearest_station(position.x, position.y)
        previous = self.robot_stations[robot_name]
        if station == previous:
            return

        self.robot_stations[robot_name] = station
        if station:
            self.station_visits[station] += 1
            self.get_logger().info(f'{robot_name} entered station {station}')
        elif previous:
            self.get_logger().info(f'{robot_name} left station {previous}')

        self.update_max_occupancy()

    def nearest_station(self, x, y):
        nearest = None
        nearest_distance = self.station_radius
        for station, pose in self.stations.items():
            distance = math.hypot(x - pose[0], y - pose[1])
            if distance <= nearest_distance:
                nearest = station
                nearest_distance = distance
        return nearest

    def current_occupancy(self):
        occupancy = {station: [] for station in self.stations}
        for robot_name, station in self.robot_stations.items():
            if station:
                occupancy[station].append(robot_name)
        return occupancy

    def update_max_occupancy(self):
        for station, robots in self.current_occupancy().items():
            self.max_occupancy[station] = max(
                self.max_occupancy[station], len(robots)
            )

    def make_occupancy(self):
        occupancy = self.current_occupancy()
        return {
            'station_occupancy': occupancy,
            'occupied_station_count': sum(bool(robots) for robots in occupancy.values()),
            'station_visits': self.station_visits,
            'max_occupancy': self.max_occupancy,
        }

    def publish_occupancy(self):
        occupancy = self.make_occupancy()
        message = String()
        message.data = json.dumps(occupancy)
        self.occupancy_publisher.publish(message)

        if self.output_path:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text(
                json.dumps(occupancy, indent=2), encoding='utf-8'
            )


def main():
    rclpy.init()
    node = StationOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_occupancy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
