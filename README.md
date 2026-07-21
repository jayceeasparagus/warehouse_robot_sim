# Multi-Robot Warehouse Simulation

A ROS 2 warehouse simulation for testing multi-robot navigation and job
scheduling. TurtleBot3 robots receive pickup and drop-off jobs, navigate between
warehouse stations, and avoid shelves and other moving robots in Gazebo.

The project includes a web dashboard for configuring experiments, creating
warehouse maps, launching the ROS 2 simulation, and reviewing performance
metrics. Experiments are saved in SQLite so completed runs can be opened again.

## System Overview

Each experiment starts a shared warehouse world and an independent Nav2 stack
for every robot. AMCL localizes each robot on the occupancy map, while laser
scans update local costmaps for obstacle avoidance. Other robots appear as
moving obstacles in these costmaps.

The dispatcher generates jobs at randomized arrival times and assigns them to
available robots. A job is complete after its robot reaches the pickup station
and then the drop-off station.

Available scheduling policies:

- `fcfs`: assigns the oldest queued job first
- `sjf`: prioritizes the shortest estimated route
- `sjf_aging`: combines route length with aging to reduce starvation
- `nearest_robot`: considers the distance between an available robot and pickup

Two monitoring nodes collect additional data during a run:

- `fleet_metrics_node`: job state counts and distance traveled per robot
- `station_occupancy_node`: station visits and current station occupancy

## Features

- One to four TurtleBot3 robots with separate ROS 2 namespaces
- Nav2 path planning, AMCL localization, and dynamic obstacle avoidance
- Five included warehouse layouts and an interactive custom-map editor
- Rotatable shelves, station checkpoints, and robot spawn points
- Randomized job arrivals with repeatable seeds
- Four dispatcher algorithms
- Live job progress, event logs, robot utilization, travel, and station metrics
- JSON, CSV, JSONL, and SQLite experiment storage
- Saved-run history in the dashboard
- RViz2 support for maps, paths, transforms, scans, and costmaps

## Requirements

- Ubuntu 24.04 or WSL2 with Ubuntu 24.04
- ROS 2 Jazzy
- Nav2
- SLAM Toolbox
- Gazebo Sim
- TurtleBot3 ROS 2 packages
- A graphical environment capable of opening Gazebo and RViz2

## Build

From the colcon workspace that contains the package:

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select warehouse_robot_sim
source install/setup.bash
```

Rebuild and source the workspace again after changing package files.

## Run the Dashboard

```bash
cd /mnt/c/Users/jayce/warehouse_robot_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run warehouse_robot_sim experiment_api
```

Open the printed address, normally:

```text
http://127.0.0.1:8080
```

### Start an Experiment

1. Select a warehouse layout.
2. Select the number of robots supported by that layout.
3. Choose a scheduler.
4. Enter the number of jobs and random seed.
5. Choose whether Gazebo should close when the run finishes.
6. Select **Start run**.

The dashboard starts Gazebo, the namespaced Nav2 stacks, monitoring nodes, and
dispatcher. During the run it displays job progress, robot utilization, recent
events, and experiment status. Final metrics remain available under
**Saved Runs**.

When **Close simulation after run** is unchecked, Gazebo remains open after all
jobs finish. Select **Stop** to close the simulation and release the ROS 2
processes.

## Create a Custom Map

Select **Create custom map** to open the 8-by-14 warehouse grid.

1. Enter a layout name.
2. Choose open floor, shelf, or a robot spawn from the grid tool.
3. Select a cell to place the item.
4. Select the same shelf or robot again to rotate it.
5. Add at least two shelves and a Robot 1 spawn.
6. Add Robot 2 through Robot 4 in numerical order when needed.
7. Select **Save layout**.

The direction shown on a shelf points toward its checkpoint. The cell in front
of a shelf must remain empty so the robot can reach the station. A robot marker
shows the robot's initial heading.

Saving a custom map generates:

- a Gazebo world
- an occupancy map and map YAML file
- pickup and drop-off station coordinates
- robot spawn poses
- a layout configuration file

Generated layouts are stored under the workspace's `custom_layouts/` directory
and immediately appear in the dashboard's warehouse layout menu.

## Metrics and Results

The dashboard reports:

- jobs generated, completed, and failed
- average queue wait time
- average job completion time
- makespan and throughput
- robot utilization and distance traveled
- station visits
- failure reasons and dispatcher events

Each experiment creates a folder under `results/` containing:

```text
config.json
summary.json
jobs.csv
events.jsonl
fleet_metrics.json
station_occupancy.json
*.log
```

Experiment settings, jobs, events, robot metrics, and station records are also
stored in `results/experiments.db`.

## Manual ROS 2 Run

The simulation can run without the dashboard. Each command should run in a
separate sourced terminal.

Launch Gazebo:

```bash
ros2 launch warehouse_robot_sim multi_robot_world.launch.py \
  layout:=standard robot_count:=3
```

Launch localization and navigation:

```bash
ros2 launch warehouse_robot_sim multi_robot_navigation.launch.py \
  layout:=standard robot_count:=3
```

Start the dispatcher:

```bash
ros2 run warehouse_robot_sim multi_robot_dispatcher_node --ros-args \
  -p use_sim_time:=true \
  -p layout:=standard \
  -p robots:="['robot1', 'robot2', 'robot3']" \
  -p scheduler_policy:=sjf_aging \
  -p max_jobs:=8 \
  -p seed:=7
```

The dashboard automatically starts the two monitoring nodes. For a manual run,
they can be started separately and publish on:

```text
/warehouse/fleet_metrics
/warehouse/station_occupancy
```

## Project Structure

```text
warehouse_robot_sim/
|-- dashboard/              Experiment dashboard
|-- config/                 Nav2 parameters
|-- launch/                 Gazebo and Nav2 launch files
|-- layouts/                Included layout configurations
|-- maps/                   SLAM and generated occupancy maps
|-- test/                   Package tests
|-- warehouse_robot_sim/    ROS 2 Python nodes
`-- worlds/                 Gazebo warehouse worlds
```
