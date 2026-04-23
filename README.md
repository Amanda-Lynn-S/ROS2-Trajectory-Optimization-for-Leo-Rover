# Rover Trajectory Optimization via Sequential Convex Programming (SCP)

This project implements a **2D rover trajectory optimization framework** using **Sequential Convex Programming (SCP)** with **collision avoidance via Signed Distance Fields (SDFs)**.  
It follows the methodology of "GuSTO: Guaranteed Sequential Trajectory Optimization via Sequential Convex Programming" (Bonalli et al., ICRA 2019) for safe, smooth, and dynamically consistent path planning. 

The system is implemented as a **ROS2 Jazzy** application split across two packages: **`rover_perception`** (point cloud processing and occupancy grid generation) and **`rover_scp`** (map bridging and trajectory optimization). Each major component runs as a ROS2 node communicating via published and subscribed topics. The perception stack produces a live occupancy grid, the map node bridges it into SDF+metadata, the planning node consumes those to run SCP, and the resulting trajectory is streamed to the rover via `/cmd_vel`.
The codebase is the ROS extension of the following project: https://github.com/Amanda-Lynn-S/Trajectory-Optimization-for-Leo-Rover

---

# Project Overview

## rover_perception package

| Module | ROS2 Role | Description |
|--------|-----------|-------------|
| **`mock_cloud_pub.py`** | **Publisher** → `/point_cloud/cloud_registered` | Publishes a synthetic PointCloud2 in the ZED optical frame for testing, simulating a flat floor with a raised bump obstacle. |
| **`cloud_to_target_frame.py`** | **Subscriber** ← `/point_cloud/cloud_registered` · **Publisher** → `/cloud_in_target_frame` | Transforms the incoming point cloud into the target frame using TF2. |
| **`height_costmap.py`** | **Subscriber** ← `/cloud_in_target_frame` · **Publisher** → `/height_costmap` | Discretizes the point cloud into a 2D grid, estimates floor height via a low percentile of Z values, and marks cells as occupied (cost=100) if their max relative height exceeds a threshold. Publishes a `nav_msgs/OccupancyGrid`. |

## rover_scp package

| Module | ROS2 Role | Description |
|--------|-----------|-------------|
| **`map_publisher_node.py`** | **Subscriber** ← `/height_costmap` · **Publisher** → `/map_meta`, `/sdf_grid` | Subscribes to the live occupancy grid from `rover_perception`. On first message received, binarizes the grid (occupied=1, free/unknown=0), computes the SDF, and publishes map metadata (dimensions, cell size, start/end grid coordinates) and the SDF array. Republishes every 2 seconds so late-starting subscribers can receive the map. Start/End positions are set via ROS2 parameters `start_x`, `start_y`, `end_x`, `end_y` (grid-cell coordinates). |
| **`rover_model.py`** | *(no ROS2 role - pure library)* | Defines the rover's continuous-time dynamics, Taylor-series linearization, and exact Zero-Order Hold (ZOH) discretization. Used internally by `Rover.py`. |
| **`Rover.py`** | *(no ROS2 role - instantiated by `rover_pp_node`)* | Defines the Rover class combining model parameters, state/control constraints, SDF interpolation, and dynamics interfaces. Constructed directly by the planning node once map data has been received. |
| **`SCP.py`** | *(no ROS2 role - instantiated by `rover_pp_node`)* | Implements the Sequential Convex Programming algorithm. Solves a convex subproblem iteratively until convergence, handling dynamics linearization/discretization, collision constraints, trust-region updates, and cost minimization. |
| **`rover_pp_node.py`** | **Subscriber** ← `/map_meta`, `/sdf_grid` · **Publisher** → `/scp_trajectory_states`, `/scp_trajectory_controls`, `/cmd_vel` | Main planning node. Waits to receive both map topics, then instantiates `Rover` and `SCP` and runs the optimizer. Publishes the full optimized state and control trajectories once. Also streams velocity commands step-by-step on `/cmd_vel` at the SCP timestep rate (`dt`). |

---

# ROS2 Topic Summary

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | `mock_cloud_pub` → `cloud_to_target_frame` | Raw point cloud in ZED optical frame. |
| `/cloud_in_target_frame` | `sensor_msgs/PointCloud2` | `cloud_to_target_frame` → `height_costmap` | Point cloud transformed into target frame. |
| `/height_costmap` | `nav_msgs/OccupancyGrid` | `height_costmap` → `map_publisher_node` | Live occupancy grid: -1=unknown, 0=free, 100=occupied. |
| `/map_meta` | `std_msgs/String` | `map_publisher_node` → `rover_pp_node` | JSON string containing map width, height, cell size, and start/end grid coordinates. |
| `/sdf_grid` | `std_msgs/Float32MultiArray` | `map_publisher_node` → `rover_pp_node` | Row-major SDF array (shape H×W) with grid dimensions encoded in the message layout. Values in grid-cell units. |
| `/scp_trajectory_states` | `std_msgs/Float32MultiArray` | `rover_pp_node` → *(logger / downstream)* | Full optimized state trajectory, shape N×5 `[px, py, θ, v, ω]`. Published once after SCP convergence. |
| `/scp_trajectory_controls` | `std_msgs/Float32MultiArray` | `rover_pp_node` → *(logger / downstream)* | Full optimized control trajectory, shape (N−1)×2 `[T_R, T_L]`. Published once after SCP convergence. |
| `/cmd_vel` | `geometry_msgs/Twist` | `rover_pp_node` → Leo Rover | Velocity commands streamed at rate `dt`. `linear.x = v`, `angular.z = ω`. |

---

# Data Flow

```
[ZED Camera / mock_cloud_pub]
  └──▶ /point_cloud/cloud_registered
          │
          ▼
  cloud_to_target_frame
  └──▶ /cloud_in_target_frame
          │
          ▼
  height_costmap
  └──▶ /height_costmap  (nav_msgs/OccupancyGrid)
          │
          ▼
  map_publisher_node
  (binarize → compute SDF → publish)
  ├──▶ /map_meta   (JSON metadata)  ──┐
  └──▶ /sdf_grid   (SDF array)      ──┤
                                      ▼
                              rover_pp_node
                              (builds Rover + SCP,
                               runs optimizer)
                                      │
                  ┌───────────────────┼───────────────────┐
                  ▼                   ▼                   ▼
     /scp_trajectory_states  /scp_trajectory_controls  /cmd_vel
       (full state traj)        (full control traj)    (Leo Rover)
```

---

# Requirements 

- Ubuntu 22.04 or Docker Desktop
- ROS 2 Jazzy

---

# Setting Up the Workspace

Source the ROS2 installation:

```bash
source /opt/ros/jazzy/setup.bash
```
Create a directory:

```bash
mkdir -p ~/ros2_ws
```
Upload the project:

```bash
cd ~/ros2_ws
git clone https://github.com/Amanda-Lynn-S/ROS2-Trajectory-Optimization-for-Leo-Rover.git
```
Install missing dependencies for packages in `src` (optional):

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

Build the workspace (from the root):

```bash
colcon build --symlink-install
```
Source the workspace:

```bash
source install/local_setup.bash
```

---

# Running the System

Launch each node in a separate terminal (source the workspace first in each terminal):

```bash
ros2 run rover_perception mock_cloud_pub
ros2 run rover_perception cloud_to_target_frame
ros2 run rover_perception height_costmap
ros2 run rover_scp map_publisher_node --ros-args -p start_x:=10 -p start_y:=10 -p end_x:=90 -p end_y:=90
ros2 run rover_scp rover_pp_node
```

`map_publisher_node` publishes immediately on receiving the first `/height_costmap` message and republishes every 2 seconds (ros arguments - start and end points - can be chosen as desired). 
`rover_pp_node` idles until it receives both `/map_meta` and `/sdf_grid`, then runs SCP automatically.
