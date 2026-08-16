# ROS 2 Multi-Robot SLAM & Patrol Fleet

A ROS 2 stack that boots a fleet of simulated robots in Gazebo, has each one run
its own SLAM + Nav2, merges their maps for visualization, and coordinates
patrol missions across the fleet through a scheduler / fleet-manager / mission
layer with heartbeat monitoring, incident reporting, and an e-stop safety
gate.

See [architecture.html](architecture.html) for a diagrammed walkthrough of the
boot sequence and the `system_readiness_supervisor` state machine. This
README covers what each package does and how to build and run the system.

## Overview

Three robots (`robot1`, `robot2`, `robot3`) spawn in a shared Gazebo world.
Each runs its own `slam_toolbox` instance and its own Nav2 stack, namespaced
per robot — there is no shared map or shared costmap. A separate node merges
the three occupancy grids into `/merged_map` for visualization only; nothing
in this repo navigates against it.

Once every robot's Nav2 lifecycle nodes are confirmed active,
`system_readiness_supervisor` publishes `/system/ready`, which unblocks the
`mission_scheduler`. The scheduler holds a priority queue of missions,
reserves an idle, sufficiently-charged robot for the head of the queue, and
asks `fleet_manager` to dispatch it. `fleet_manager` aggregates per-robot
heartbeats into fleet state, starts/stops missions via each robot's
`patrol_manager` (which drives Nav2's `FollowWaypoints` action), and exposes
e-stop and incident-reporting services. A `safety_gate` node sits between
Nav2's velocity output and the robot base on every robot so an e-stop takes
effect regardless of what the mission or scheduling layers are doing.

## Packages

| Package | Language | Purpose |
|---|---|---|
| [bringup](src/bringup) | C++ / launch | Top-level launch orchestration; `system_readiness_supervisor` gates the rest of the stack on confirmed Nav2 lifecycle state per robot. |
| [sim](src/sim) | launch | Gazebo Harmonic simulation — spawns the fleet, bridges Gazebo↔ROS topics via `ros_gz_bridge`. |
| [description](src/description) | Xacro/URDF | Robot model (URDF, Xacro, meshes) for the `patrol_robot` platform. |
| [mapping](src/mapping) | C++ / launch | Per-robot `slam_toolbox` bringup, map-frame management, and `pose_jump_watchdog` — a cheap proxy for localization confidence that flags a teleporting `map→base_link` transform as "not ok". |
| [merge_map](src/merge_map) | Python | TF-aware online occupancy-grid merger; publishes `/merged_map` for visualization (no in-repo subscriber). Also ships an offline merge tool. |
| [navigation](src/navigation) | launch/config | Namespaced Nav2 bringup and per-robot parameter files. |
| [patrol](src/patrol) | C++ | `patrol_manager` — per-robot waypoint-loop patrol driven through Nav2's `FollowWaypoints` action, with start/stop services. |
| [fleet](src/fleet) | C++ | `fleet_manager_node` (heartbeat aggregation, mission dispatch, e-stop, incident reporting, SQLite-backed mission/incident log) and `robot_heartbeat_node` (per-robot state aggregator). |
| [scheduler](src/scheduler) | C++ | `mission_scheduler_node` — fleet-aware priority mission queue; reserves an idle, charged robot and hands it to `fleet_manager`. |
| [safety](src/safety) | C++ | `safety_gate_node` — forces `cmd_vel` to zero on e-stop, otherwise passes Nav2's `cmd_vel_nav` straight through. |
| [interfaces](src/interfaces) | msg/srv | Custom messages and services shared across the stack. |

## Custom interfaces

**Messages:** `RobotHeartbeat`, `FleetRobotState`, `FleetState`,
`MissionQueueState`, `RobotIncident`

**Services:** `QueueMission`, `CancelMission`, `AssignMission`,
`StartMission`, `ReportIncident`

Definitions live in [interfaces](src/interfaces/msg) and
support five mission types: `patrol`, `go_to`, `return_home`, `charge`,
`inspect`.

## Key topics and services

| Name | Type | Notes |
|---|---|---|
| `/system/ready` | `std_msgs/Bool` | Latched; gates the scheduler. Published by `system_readiness_supervisor` once every robot's Nav2 lifecycle nodes are confirmed `ACTIVE`. |
| `/robotN/map`, `/merged_map` | `nav_msgs/OccupancyGrid` | Per-robot SLAM output, and the visualization-only merge. |
| `/robotN/heartbeat` | `RobotHeartbeat` | Per-robot state, battery, current mission, localization health. |
| `/fleet/state` | `FleetState` | Aggregated fleet view, published by `fleet_manager`. |
| `/scheduler/queue_mission`, `/scheduler/cancel_mission` | `QueueMission`, `CancelMission` | Entry points for submitting/cancelling work. |
| `/scheduler/queue_state` | `MissionQueueState` | Latched snapshot of the pending queue. |
| `/fleet/assign_mission` | `AssignMission` | Scheduler → fleet manager dispatch (start/stop a robot on a mission). |
| `/fleet/estop_all`, `/fleet/estop_release_all` | `std_srvs/Trigger` | Fleet-wide e-stop control. |
| `/fleet/report_incident` | `ReportIncident` | Logged to SQLite; auto-dispatches a mission above a configurable severity. |
| `/robotN/patrol/start`, `/robotN/patrol/stop` | `StartMission`, `Trigger` | Per-robot mission control, called by `fleet_manager`. |

## Prerequisites

- ROS 2 (with `ament_cmake` / `ament_python`)
- Gazebo Harmonic (`ros_gz_sim`, `ros_gz_bridge`)
- Nav2 (`nav2_msgs`, `nav2_lifecycle_manager`, ...)
- `slam_toolbox`
- SQLite3 development headers (`libsqlite3-dev`) — used by `fleet`'s mission/incident store

## Build

```bash
cd ~/ros2-mutli-robot-slam
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Bring up the full stack — Gazebo, safety gates, SLAM, map merge, Nav2,
patrol, scheduler, and the readiness supervisor:

```bash
ros2 launch bringup full_system.launch.py
```

Wait for `/system/ready` to latch `true` before queuing missions:

```bash
ros2 topic echo /system/ready --once
```

Queue a patrol mission:

```bash
ros2 service call /scheduler/queue_mission interfaces/srv/QueueMission \
  "{mission_type: 'patrol', priority: 10, has_target: false}"
```

Emergency-stop the fleet:

```bash
ros2 service call /fleet/estop_all std_srvs/srv/Trigger {}
```

Smaller launch files are available for bringing up subsystems individually
(see `launch/` under [bringup](src/bringup/launch),
[mapping](src/mapping/launch),
[navigation](src/navigation/launch),
[patrol](src/patrol/launch), and
[sim](src/sim/launch)) — useful when iterating on one
layer without restarting Gazebo.

## Design notes

- **Readiness gating is layered, not a final check.** `/system/ready` only
  unblocks the scheduler, which sits upstream of the fleet and patrol layers
  — it is not a check that runs after everything is already dispatching
  missions. See Figure 2 in [architecture.html](architecture.html) for the
  per-robot state machine that decides when it's safe to flip that bit.
- **Localization confidence is a proxy, not a real metric.**
  `pose_jump_watchdog` doesn't measure SLAM covariance — `slam_toolbox`'s
  online async mode doesn't expose one the way AMCL does. It instead flags
  `map→base_link` jumps larger than a robot could physically move between
  checks. A steady lock means the map hasn't visibly jumped, not that it's
  metrically correct.
- **The safety gate is topology, not logic.** `safety_gate_node` is a thin
  splice between Nav2's `cmd_vel_nav` output and the robot's `cmd_vel` input
  — e-stop takes effect regardless of what the mission, fleet, or scheduling
  layers are doing, because it doesn't go through them.
- **Map merge is for visualization only.** Each robot plans against its own
  `/robotN/map`; `/merged_map` has no subscriber in this repo.
