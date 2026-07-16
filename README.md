# Multi-Robot Warehouse Simulation

This project is a ROS 2 warehouse robot simulation where TurtleBots complete pickup and dropoff jobs in Gazebo. Each robot uses Nav2 for navigation, AMCL for localization, and RViz2 for visualizing maps, paths, costmaps, and robot behavior.

The project also includes a small experiment dashboard. The dashboard lets a user choose a warehouse layout, number of robots, dispatcher policy, number of jobs, and random seed. After the run finishes, it shows scheduling and navigation metrics.

## Features

- Configurable simulation with up to four TurtleBots in Gazebo
- Nav2 navigation with robot namespaces
- Static warehouse maps and generated benchmark layouts
- Dynamic obstacle behavior through local Nav2 costmaps
- Web dashboard for running experiments
- Dispatcher policies: FCFS, shortest-job-first, SJF with aging, and nearest robot
- Metrics for completed jobs, failed jobs, wait time, completion time, makespan, and utilization

## Build

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the build again after changing Python files, launch files, maps, layouts, worlds, or dashboard files.

## Run the Dashboard

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run warehouse_robot_sim experiment_api
```

Open the dashboard in a browser:

```text
http://127.0.0.1:8080
```

From the dashboard, choose:

- warehouse layout
- robot count
- scheduling policy
- job count
- random seed

Then press **Start**. The backend launches the simulation, runs the dispatcher, records events, and updates the dashboard with final metrics.

## Layouts

Available layouts:

- `standard`
- `wide_aisle`
- `asymmetric`
- `compact`
- `cross_traffic`

The generated layouts are meant for cleaner testing, easier demos, and repeatable benchmarking. The original SLAM-created map files are still kept in the project.

## Dispatcher Policies

Available policies:

- `fcfs`: first-come, first-served
- `sjf`: shortest-job-first
- `sjf_aging`: shortest-job-first with aging to reduce starvation
- `nearest_robot`: assigns jobs based on robot distance when possible

## Results

Each dashboard experiment creates a folder in:

```text
/mnt/c/Users/jayce/warehouse_robot_sim/results/
```

Each result folder contains:

- `config.json`: experiment settings
- `summary.json`: final metrics
- `jobs.csv`: job timing and outcome data
- `events.jsonl`: dispatcher event log
- ROS process logs for the world, navigation, and dispatcher

## Manual Run

The simulation can also be launched without the dashboard.

Terminal 1:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch warehouse_robot_sim multi_robot_world.launch.py layout:=standard robot_count:=3
```

Terminal 2:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch warehouse_robot_sim multi_robot_navigation.launch.py layout:=standard robot_count:=3
```

Terminal 3:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run warehouse_robot_sim multi_robot_dispatcher_node --ros-args \
  -p use_sim_time:=true \
  -p layout:=standard \
  -p robots:="['robot1', 'robot2', 'robot3']" \
  -p scheduler_policy:=sjf_aging \
  -p max_jobs:=8 \
  -p seed:=7
```

## Project Structure

```text
warehouse_robot_sim/
+-- dashboard/              Web experiment dashboard
+-- layouts/                Layout configs and station coordinates
+-- launch/                 Gazebo and Nav2 launch files
+-- maps/                   SLAM and generated occupancy maps
+-- worlds/                 Gazebo warehouse worlds
+-- warehouse_robot_sim/    ROS 2 Python nodes
```
