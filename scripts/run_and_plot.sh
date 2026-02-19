#!/bin/bash

# 1. Setup the environment
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle

# Clean up any stuck background processes first
sudo pkill -9 -f ros
sudo pkill -9 -f gazebo

echo "========================================="
echo "🚀 STARTING SIMULATION IN BACKGROUND..."
echo "========================================="
# Run the launch file in the background using the '&' symbol
ros2 launch assignment_core assignment_launch.py &
LAUNCH_PID=$!

echo "⏳ Waiting 8 seconds for Gazebo and RViz to load..."
sleep 8

echo "========================================="
echo "📈 STARTING THE PLOTTER..."
echo "========================================="
# Run the Python plotter in the foreground
python3 src/assignment_core/src/plot_data.py

echo "✅ Plotting finished! You can save your graphs."
echo "Press Ctrl+C to close Gazebo and RViz when you are done."

# Wait for the user to press Ctrl+C to kill the simulation
wait $LAUNCH_PID
