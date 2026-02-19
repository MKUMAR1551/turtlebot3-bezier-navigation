Autonomous TurtleBot3 Navigation: Bezier Curve Smoothing and State-Machine Control

Author: Mehulkumar Prasad

Domain: Robotics and Automation

Environment: ROS 2 (Jazzy), Gazebo Sim, RViz2, Python 3, C++17
1. Project Overview

This project implements an autonomous navigation stack for a TurtleBot3 (Waffle). The objective is to drive the robot along a predefined square trajectory (2m×2m) while mathematically smoothing the sharp 90-degree corners using Quadratic Bezier curves. The system features a custom closed-loop controller with a state-machine architecture to ensure precise stopping at the final destination without oscillation or infinite rotational loops.
2. Mathematical Path Generation (Path Planner)

Standard square waypoints cause kinematic instability at the corners for differential drive robots. To solve this, the path_planner_node applies a Quadratic Bezier interpolation to generate a continuous, differentiable trajectory.

Given a start point P0​, an end point P2​, and a calculated control point P1​, the position vector B(t) for any parameter t where 0≤t≤1 is defined as:
B(t)=(1−t)2P0​+2(1−t)tP1​+t2P2​

The control point P1​ is dynamically calculated to push the curve slightly outward, ensuring a smooth transition radius between the linear segments of the square. The generated trajectory is published as a nav_msgs/msg/Path on the /smooth_path topic.
3. Kinematic Control and Logic (Controller Node)

The controller_node acts as the "brain" of the robot, utilizing a look-ahead logic (similar to Pure Pursuit) and a discrete state machine.
Path Tracking

The robot subscribes to its own odometry (/odom) and compares its position against the target path. It looks 10 indices ahead of its closest path point to calculate the heading error (θerr​):
θerr​=arctan2(ytarget​−yrobot​,xtarget​−xrobot​)−θrobot​

The controller utilizes an adaptive velocity profile. Linear velocity (vx​) operates at a cruising speed of 0.15 m/s, but dynamically decelerates to 0.05 m/s when ∣θerr​∣>0.4 rad to stabilize the turns. Angular velocity (ωz​) is controlled proportionally to the heading error.
State Machine & The "Closed-Loop" Fix

Because the path is a closed square (starting and ending at 0,0), a standard distance-to-goal check fails (the distance is zero at the start, triggering a false stop). To solve this, the node implements a State enum (FOLLOWING, FINISHED) and a memory flag:

    has_left_start_ Flag: The system monitors the Euclidean distance from the origin: x2+y2​. Once this exceeds 1.0 meter, the flag is permanently set to true.

    The Kill-Switch: When the distance to the final waypoint falls below 0.4 meters and the has_left_start_ flag is true, the state transitions to FINISHED. All kinematic calculations halt, and vx​=0,ωz​=0 is continuously published to lock the motors.

4. Build and Execution Instructions
Prerequisites

Ensure the TurtleBot3 simulation packages are installed:
Bash

sudo apt update
sudo apt install ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-msgs

Building the Workspace
Bash

cd ~/turtlebot3_ws
colcon build --packages-select assignment_core
source install/setup.bash

Running the System

To execute the full demonstration, capture the plotting data, and visualize the path, use the automated bash scripts provided in the workspace.

Terminal 1: Launch the Simulation and Data Plotter
Bash

cd ~/turtlebot3_ws
./run_and_plot.sh

Note: This script will launch Gazebo and the ROS 2 nodes in the background, wait 8 seconds for the environment to load, and then automatically record /odom and /cmd_vel data for 65 seconds to generate the performance graphs.

Terminal 2: Launch RViz2 Visualization
Bash

cd ~/turtlebot3_ws
./launch_rviz.sh

Note: Ensure the Fixed Frame is set to odom to view the published /smooth_path topic.
5. Results and Analysis

The system successfully navigates the 2m×2m area.
## Navigation Demo
<div align="center">
  <h3>Project Demonstration</h3>
  <video src="https://github.com/MKUMAR1551/turtlebot3-bezier-navigation/raw/main/turtlebot.mp4" width="100%" controls>
    Your browser does not support the video tag.
  </video>
</div>
<img width="1199" height="873" alt="Screenshot from 2026-02-19 13-19-16" src="https://github.com/user-attachments/assets/e1ab079e-3bb1-4422-b280-94fdd057beb1" />
The generated plots confirm the controller's effectiveness. The spatial trajectory shows perfect corner smoothing, while the velocity profile graph verifies the adaptive speed reduction during cornering and the hard stop upon reaching the FINISHED state.
