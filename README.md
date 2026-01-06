# Assignment 2 - Research Track - Safe Robot Teleoperation (ROS2)

This project is a practical exercise to explore advanced ROS2 concepts, including custom messages, custom services, LaserScan processing, and safety-aware teleoperation.
Users can manually control a robot while a safety node continuously monitors the environment and prevents collisions through automatic rollback behavior.

The project demonstrates real-time interaction between nodes, service-based configuration, and safety logic for autonomous correction.

## Table of Contents
1. [Description](#description)
2. [Features](#features)
3. [Technologies](#technologies)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Diagram](#diagram)

## Description

Users can manually control a robot through a terminal-based interface. The system consists of two main nodes:
- User Interface Node (`node1.cpp`): Sends raw velocity commands and interacts with custom services.
- Safety Node (`node2.cpp`): Processes LaserScan data, enforces safety constraints, and publishes filtered velocity commands.

Motion commands are sent using `geometry_msgs/Twist` messages, while obstacle detection is performed using `sensor_msgs/LaserScan`.

**Implemented Safety Controls:**
- Obstacle detection:  
The LaserScan is divided into left, front, and right sectors. The minimum distance is computed and published at 10 Hz.
- Automatic rollback:  
If the robot moves too close to an obstacle (distance < threshold), a rollback maneuver is triggered:
  - If moving forward → move backward
  - If moving backward → move forward
  - If rotating → rotation is allowed (no rollback)
- Threshold configuration:  
Users can update the safety threshold at runtime via a custom service.
- Velocity averaging:  
The last 5 velocity commands are stored, and their average can be retrieved via a service.

After rollback is completed, the robot is automatically stopped.

## Features

- **Feature 1: User interaction (`node1.cpp`)**

   - Terminal-based teleoperation using single-key commands.
   - Adjustable linear and angular speeds.
   - Sends raw velocity commands via a publisher (`cmd_vel_raw`).
   - Interacts with two custom services:
     -  `set_threshold` → update safety threshold.
     -  `get_vel_avg` → retrieve average of last 5 velocity commands.
   - Input validation prevents invalid or non-numeric entries.

   **Example interaction:**

	   w - Forward
      s - Backward
      a - Rotate left
      d - Rotate right
      x - Stop
      t - Set new threshold
      m - Get averages of last 5 velocities
      l - Set linear speed
      r - Set angular speed
      q - Quit
	
- **Feature 2: Safety controls (`node2.cpp`)**

  - Real-time processing of LaserScan data.
  - Division of scan into left, front, and right sectors.
  - Publishes obstacle information `/obstacle_info` using a custom message.
  - Enforces safety threshold with rollback behavior.
  - Publishes filtered velocity commands `/cmd_vel`.
  - Maintains a queue of the last 5 velocity commands.
  - Provides two custom services: `SetThreshold.srv` and `GetVelAvg.srv`.
  - Timer-based loop publishes obstacle information at 10 Hz.

## Technologies 

- **Languages:** C++
- **Framework:** ROS2 (Robot Operating System 2)
- **Additional tools:** Docker, GitHub
- **Used packages:**
    - `geometry_msgs/Twist`: velocity commands.
    - `sensor_msgs/LaserScan`: obstacle detection.
    - `roscpp`: ROS2 C++ client library.
    - `rosidl_default_generators`: custom message/service generation.
    - Custom interfaces: `ObstacleInfo.msg`, `SetThreshold.srv`, `GetVelAvg.srv`.

## Installation

Follow these steps to set up the project:

1. Build the project: 

    - `colcon build`
    - `source install/setup.bash`

2. Launch processes in separate terminals:

	  - **Terminal 1**: Launch your robot simulation (if applicable)
    - **Terminal 2**: Launch the user interface node: `ros2 run assignment2 node1`
	- **Terminal 3**: Launch the safety node: `ros2 run assignment2 node2`

## Usage

This project provides an interactive way to explore ROS2 concepts such as:

  - Real-time Publisher/Subscriber communication.

  - Custom message and service creation.

  - LaserScan processing and obstacle detection.

  - Safety and collision prevention through rollback logic.

  - Timer-based periodic publishing.

  - User-terminal interaction with input validation.

It is suitable for students and developers who want to deepen their understanding of ROS2 node design, safety mechanisms, and multi-node communication in a practical scenario.

## Diagram
The following diagram illustrates the overall architecture of the system, showing how the two nodes interact through topics and services, and how the safety logic integrates LaserScan data with user commands.

                           +-----------------------------+
                           |     user_interface_node     |
                           |           (node1)           |
                           +-----------------------------+
                           | Publishes: /cmd_vel_raw     |
                           | Calls: /set_threshold (srv) |
                           | Calls: /get_vel_avg (srv)   |
                           +--------------+--------------+
                                          |
                                          | raw velocity commands
                                          v
                           +-----------------------------+
                           |        safety_node          |
                           |           (node2)           |
                           +-----------------------------+
                           | Subscribes: /cmd_vel_raw    |
                           | Subscribes: /scan           |
                           | Publishes: /cmd_vel         |
                           | Publishes: /obstacle_info   |
                           | Provides: set_threshold     |
                           | Provides: get_vel_avg       |
                           +--------------+--------------+
                                          |
                                          | filtered velocity commands
                                          v
                                 +------------------+
                                 |     Robot        |
                                 | (simulation or   |
                                 |  real hardware)  |
                                 +------------------+

- `user_interface_node`: Sends raw velocity commands and interacts with the safety node through two custom services.
- `safety_node`: Processes LaserScan data, enforces safety constraints, and publishes safe velocity commands.
  It also publishes obstacle information and provides two services for configuration and monitoring.
- Robot: Receives the final, safety-filtered velocity commands.
