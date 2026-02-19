import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the Turtlebot3 Simulation
    pkg_gazebo_ros = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        # --- TASK 1: Start the Simulation (Gazebo) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'empty_world.launch.py')
            ),
        ),

        # --- TASK 2: Start the Bridge (CRITICAL for Jazzy) ---
        # This connects ROS 2 to Gazebo so the robot can move
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        # --- TASK 3: Run Your Algorithms ---
        
        # Node 1: Path Smoother (Generates the Green Curve)
        Node(
            package='assignment_core',
            executable='path_planner_node',
            name='path_planner',
            output='screen'
        ),

        # Node 2: Trajectory Generator (Adds time/velocity to the curve)
        Node(
            package='assignment_core',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='screen'
        ),

        # Node 3: Controller (Drives the robot)
        Node(
            package='assignment_core',
            executable='controller_node',
            name='controller',
            output='screen'
        ),
    ])
