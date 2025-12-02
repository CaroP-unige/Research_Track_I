# Assignment 1 - Research Track - Turtle Motions (ROS2)

This project is a practical exercise to explore fundamental concepts of ROS2 (Robot Operating System 2) using the `turtlesim` simulator. Users can interactively control two turtles and understand Publisher/Subscriber communication, timers, and real-time user input handling in a ROS2 environment.

The project allows users to manually control the movement of turtles while ensuring safety constraints, such as maintaining workspace boundaries and a minimum distance between turtles.

## Table of Contents
1. [Description](#description)
2. [Features](#features)
3. [Technologies](#technologies)
4. [Installation](#installation)
5. [Usage](#usage)

## Description

Users can move one of the two turtles in the workspace by specifying:
- Which turtle to move (`turtle1` or `turtle2`).
- The movement direction (rotate right, rotate left, forward, backward).
- The speed of movement (linear or angular).

Motion commands are sent using `geometry_msgs/Twist` messages, while the positions of both turtles are monitored through `turtlesim/Pose messages`.

**Implemented Safety Controls:**
- Turtles cannot exit the workspace boundaries (1.0 ≤ x ≤ 10.0). If they attempt to, a corrective motion is applied to move them back.
- Turtles maintain a minimum safety distance (default = 1.0 unit). If turtles are too close, corrective motion is applied to separate them.

In both cases, after corrective action, the velocity is set to zero to stop the turtles.

## Features

- **Feature 1: User interaction (`node1.cpp`)**

   - Allows the user to select which turtle to move, direction, and speed.
   - Input is read from the terminal and validated to prevent invalid commands.
   - Movement commands are sent via `Publisher` objects.
   - Each movement lasts 1 second and then a stop command is automatically published.
   - Input validation prevents non-numeric or out-of-range entries.

   **Example interaction:**

	  Which turtle do you want to move? (1=turtle1, 2=turtle2, 0=exit): 
	------------------------------------
	  Direction? (1=rotate right, 2=rotate left, 3=move forward, 4=move backward): 
	------------------------------------
	  Speed (>0): 
	
  
- **Feature 2: Safety controls (`node2.cpp`)**

  - Real-time monitoring of turtle positions using `turtlesim/Pose` subscribers.
  - Corrective actions: 
            - *Workspace boundaries:* Turtles are prevented from leaving the simulation window; corrective motion is applied.
            - *Minimum distance:* If turtles get too close, they are separated using corrective velocities.
  - Distance between turtles is published on a dedicated topic (`/distance`) using `std_msgs/Float32`.
  - Motion corrections use `geometry_msgs/Twist` publishers for each turtle.
  - Timer-based loop ensures continuous monitoring without blocking the user input in `node1`.

## Technologies 

- **Languages:** C++
- **Framework:** ROS2 (Robot Operating System 2)
- **Additional tools:** Docker, GitHub
- **Used packages:**
    - `turtlesim`: to simulate the turtles and the workspace.
    - `geometry_msgs/Twist`: to handle velocity messages.
    - `turtlesim/Pose`: provides turtle positions for monitoring and safety
    - `roscpp`: to implement ROS nodes in C++.
    - `std_msgs/Float32`: used to publish the distance between turtles.

## Installation

Follow these steps to set up the project:

1. Clone the repository:

	- `git clone https://github.com/CaroP-unige/Research_Track_I.git`
	- `cd Assignment1`

2. Build the project: 

    - `colcon build`
    - `source install/setup.bash`

3. Launch processes in separate terminals:

	- **Terminal 1**: Launch the turtlesim simulator: `ros2 run turtlesim turtlesim_node`
    - **Terminal 2**: Spawn the second turtle (using the dedicated Python script not included in the package): `python3 turtle_spawn.py`
	- **Terminal 3**: Launch node1: `ros2 run Assignment1 node1`
	- **Terminal 4**: Launch node2: `ros2 run Assignment1 node2`

## Usage

This project provides an interactive way to explore ROS2 concepts:

- Real-time Publisher/Subscriber communication.
- User-terminal interaction with input validation.
- Safety and collision management in a simulated environment.
- Timer-based periodic control loops to monitor turtle positions.

It is suitable for beginners and intermediate users who want to understand ROS2 node design, messaging, and control strategies in a practical setting.