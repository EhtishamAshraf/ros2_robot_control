# ROS 2 Robot Control

A ROS 2 Humble project for safe mobile robot teleoperation in Gazebo.

The user provides linear and angular velocity commands through the `velocity_controller`. The `safety_monitor` uses LiDAR data to monitor the closest obstacle and automatically moves the robot back when the safety threshold is violated.

### Features

Tested with **Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress**.

- Linear and angular velocity control
- LiDAR-based obstacle monitoring
- Automatic safety recovery
- Runtime safety-threshold configuration
- Custom obstacle information message
- Average of the latest 5 velocity commands

### Run

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

In separate terminals:

```bash
ros2 run robot_safety safety_monitor
ros2 run robot_safety velocity_controller
```

Change the safety threshold:

```bash
ros2 service call /set_threshold robot_interfaces/srv/SetThreshold "{threshold: 0.8}"
```

Get the average of the latest velocity commands:

```bash
ros2 service call /get_average_velocity robot_interfaces/srv/GetAverageVelocity "{}"
```

Monitor the closest obstacle:

```bash
ros2 topic echo /obstacle_info
```