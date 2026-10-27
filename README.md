# MPPI-Based Navigation

## Overview

This project replaces the Pure Pursuit tracking approach with an MPPI-style controller for a ROS 2 mobile robot. The controller uses a planned path, odometry, and 2D LiDAR measurements to select velocity commands while considering obstacle clearance and motion smoothness.

### Implemented components

- Cost-weighted sampling of candidate control sequences.
- Local target selection along the planned path.
- LiDAR scan processing and transformation into the `odom` frame using TF.
- Obstacle penalties during predicted trajectory evaluation.
- Recovery candidate evaluation when normal sampling does not produce a valid sequence.
- Velocity command publishing to `/cmd_vel`.

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo simulation
- C++

### ROS topics

| Topic | Message type |
|---|---|
| `/path` | `nav_msgs/msg/Path` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` |

Confirm the exact Gazebo version and dependencies in the environment used for the final demonstration.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch nav sim_bringup.py
```

## Running the Simulation

1. Start Gazebo using the launch command configured in your workspace.
2. Start the path-generation and smoothing components required by your project.
3. Start the MPPI controller using your configured launch setup.
4. Confirm `/path`, `/odom`, and `/scan` are publishing and that TF can transform the LiDAR frame into `odom`.
5. Observe the robot and `/cmd_vel` during each test.



## Test Summary

The robot was tested in Gazebo. The reported results are that obstacle avoidance and rejoining the original path both work.

Record only scenarios actually tested:

- [ ] Path following
- [ ] Static cylinder obstacle avoidance
- [ ] Rejoining the original path
- [ ] Dynamic-obstacle response, if tested
- [ ] Safe stopping when no acceptable recovery candidate is available, if tested

## Controller Parameters

These are the code's default values; runtime configuration may override them.

| Parameter | Default | Description |
|---|---:|---|
| `samples` | 300 | Number of sampled control sequences |
| `horizon_steps` | 20 | Predicted steps per sequence |
| `dt` | 0.1 s | Prediction time step |
| `max_linear` | 0.18 m/s | Maximum forward speed |
| `max_angular` | 1.0 rad/s | Maximum angular speed |
| `obstacle_margin` | 0.45 m | Preferred obstacle-clearance margin |
| `lookahead_distance` | 0.45 m | Distance used to select a path target |
| `goal_tolerance` | 0.20 m | Goal-completion distance threshold |
| `mppi_lambda` | 10.0 | Exponential cost-weighting parameter |
| `noise_linear_std` | 0.06 | Linear sampling-noise standard deviation |
| `noise_angular_std` | 0.35 | Angular sampling-noise standard deviation |

## Limitations

- The controller uses a simplified point-based representation of LiDAR obstacles.
- It reacts to updated LiDAR measurements but does not explicitly estimate each moving obstacle's velocity or predict its future trajectory.
- Sampling and recovery checks do not provide a formal guarantee of collision-free motion. Results depend on sensor data, transforms, parameter values, and test conditions.

## Demonstration Video

[Watch the Gazebo demonstration video](https://drive.google.com/file/d/1RSF3C_x0djRSXUE1xoCxx9xtB7YZx9BD/view?usp=sharing)

## Submission Checklist

- [ ] Final `mppi_controller.cpp` included
- [ ] Exact launch commands verified and documented
- [ ] Build output captured
- [ ] Gazebo demonstration video linked
- [ ] Test results recorded accurately
- [ ] Only relevant source and configuration files included
