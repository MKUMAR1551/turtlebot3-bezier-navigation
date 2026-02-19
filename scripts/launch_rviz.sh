#!/bin/bash

# Setup ROS 2 and Workspace
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Set TurtleBot3 Model
export TURTLEBOT3_MODEL=waffle

echo "Opening RViz2..."

# Launch RViz with the TurtleBot3 default view
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_viz)/share/turtlebot3_viz/rviz/model.rviz
