# Robot-Localization-and-Control-Simulation
Vehicle Localization and Control Simulation based on non-holonomic kinematics based on bicycle model

## Description

This project simulates a 2D robotic vehicle using a Controller for autonomous steering. The vehicle operates in both a global and local frame, navigating through a virtual world using feedback from its position and orientation.

The simulation includes:
- **Real-world dynamics**: Continuous updates of the robot’s position, velocity, and steering.
- **Localization**: A low-frequency sensor-like localization update.
- **Stanley Controller**: A control algorithm used in autonomous driving to correct the robot's path based on cross-track and heading errors.
- **OpenCV Visualization**: Displays the robot's movement using OpenCV to track its local and global positions.
- **Control noise simulation**: Adds steering noise to simulate real-world control imperfections.

The project is written in Python and uses libraries such as `numpy`, `opencv`.

---

## Features

- **Edited Stanley Controller**: Adjusts the steering angle based on cross-track and heading errors to follow a desired path.
- **Real-time visualization**: Displays the robot’s path and position using OpenCV.
- **Noise simulation**: Adds randomness to steering inputs to simulate real-world vehicle dynamics.
- **Feedback mechanism**: Uses both position and orientation feedback for control.
- **Configurable parameters**: Modify velocity, steering noise, and control gains in the code to observe different behaviors.
