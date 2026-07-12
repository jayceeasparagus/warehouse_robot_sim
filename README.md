# Warehouse Robot Simulator

This is a ROS 2 project where TurtleBot robots drive around a small warehouse, receive pickup/dropoff jobs, and avoid shelves and other robots using Nav2.

The main goal was to learn ROS 2, Gazebo, RViz, Nav2, AMCL, and SLAM while also adding some software-engineering logic through a custom job dispatcher.

## What It Does

- Runs a custom warehouse world in Gazebo
- Spawns three TurtleBots: `robot1`, `robot2`, and `robot3`
- Uses Nav2 to move each robot to warehouse stations
- Uses local costmaps and laser scans for obstacle avoidance
- Uses a Python dispatcher node to assign jobs to available robots
- Supports random jobs or a fixed job sequence for testing
- Uses a basic priority system:
  - shorter jobs are preferred
  - waiting jobs slowly gain priority so they do not starve
  - robots closer to a pickup are slightly preferred

## Main Files

```text
warehouse_robot_sim/
  launch/
    multi_robot_world.launch.py
    multi_robot_navigation.launch.py

  config/
    nav2_params_robot1.yaml
    nav2_params_robot2.yaml
    nav2_params_robot3.yaml

  maps/
    warehouse_map.yaml          # map made with SLAM
    clean_warehouse_map.yaml    # cleaner generated map for testing

  worlds/
    warehouse.world

  warehouse_robot_sim/
    multi_robot_dispatcher_node.py
    initial_pose_publisher_node.py
```

## Warehouse Stations

```text
A1     A2     A3

   driving aisle

B1     B2     B3
```

The stations are used as pickup and dropoff points.

## Build

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Use three terminals.

Terminal 1:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch warehouse_robot_sim multi_robot_world.launch.py
```

Terminal 2:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch warehouse_robot_sim multi_robot_navigation.launch.py
```

Terminal 3:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run warehouse_robot_sim multi_robot_dispatcher_node --ros-args -p use_sim_time:=true
```

Example fixed test:

```bash
ros2 run warehouse_robot_sim multi_robot_dispatcher_node --ros-args -p use_sim_time:=true -p max_jobs:=6 -p job_sequence:="['A1:B3', 'A3:B1', 'A2:B2', 'B1:A3', 'B3:A1', 'A1:B1']"
```

## Checking Avoidance

In RViz, add:

```text
/robot1/local_costmap/costmap
/robot2/local_costmap/costmap
/robot3/local_costmap/costmap
/robot1/scan
/robot2/scan
/robot3/scan
```

If another robot shows up as a black dot/blob in the local costmap, Nav2 is seeing it as an obstacle.

For RViz TF issues, launch RViz for one robot like this:

```bash
rviz2 --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static -p use_sim_time:=true
```

Replace `robot1` with `robot2` or `robot3` if needed.

## Notes

I kept both maps:

- `warehouse_map.yaml` was made using SLAM Toolbox
- `clean_warehouse_map.yaml` is a generated map that makes Nav2 testing more consistent

The dispatcher is intentionally simple, but it works like a small scheduler for warehouse jobs.
