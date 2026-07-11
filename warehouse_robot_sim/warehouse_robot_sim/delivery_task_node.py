import math
import sys
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
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


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class DeliveryTaskNode(Node):
    def __init__(self, pickup: str, dropoff: str):
        super().__init__('delivery_task_node')
        self.pickup = pickup.upper()
        self.dropoff = dropoff.upper()
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def run(self) -> bool:
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 navigate_to_pose action server is not available.')
            return False

        if not self.navigate_to_station(self.pickup, 'pickup'):
            return False

        self.get_logger().info(f'Pickup complete at {self.pickup}. Pausing for 2 seconds.')
        rclpy.spin_once(self, timeout_sec=2.0)

        if not self.navigate_to_station(self.dropoff, 'dropoff'):
            return False

        self.get_logger().info(f'Delivery complete: {self.pickup} -> {self.dropoff}')
        return True

    def navigate_to_station(self, station: str, label: str) -> bool:
        pose = WAYPOINTS.get(station)
        if pose is None:
            valid = ', '.join(sorted(WAYPOINTS))
            self.get_logger().error(f'Unknown {label} station {station}. Valid stations: {valid}')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(*pose)

        x, y, yaw = pose
        self.get_logger().info(
            f'Sending {label} goal {station}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label.capitalize()} goal {station} was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error(f'{label.capitalize()} goal {station} returned no result.')
            return False

        status = result.status
        if status != 4:
            self.get_logger().error(f'{label.capitalize()} goal {station} failed with status {status}.')
            return False

        self.get_logger().info(f'Reached {label} station {station}.')
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


def parse_args():
    args = sys.argv[1:]
    if len(args) != 2:
        valid = ', '.join(sorted(WAYPOINTS))
        print('Usage: ros2 run warehouse_robot_sim delivery_task_node PICKUP DROPOFF')
        print(f'Valid stations: {valid}')
        raise SystemExit(2)
    return args[0], args[1]


def main():
    pickup, dropoff = parse_args()
    rclpy.init()
    node = DeliveryTaskNode(pickup, dropoff)
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
