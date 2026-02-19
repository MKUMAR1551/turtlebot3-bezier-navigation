import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import os

class DataPlotter(Node):
    def __init__(self):
        super().__init__('data_plotter')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        
        self.times = []
        self.x_data = []
        self.y_data = []
        self.v_data = []
        self.start_time = time.time()
        
        self.get_logger().info("📡 Collecting data for 65 seconds... Make sure the robot is moving!")

    def odom_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def vel_callback(self, msg):
        self.v_data.append(msg.linear.x)
        self.times.append(time.time() - self.start_time)

def main(args=None):
    rclpy.init(args=args)
    plotter = DataPlotter()
    
    # Spin and collect data for exactly 15 seconds
    start = time.time()
    while rclpy.ok() and (time.time() - start) < 65.0:
        rclpy.spin_once(plotter, timeout_sec=0.1)
    
    plotter.get_logger().info("🛑 Finished collecting data. Generating plots...")
    
    # Safety check
    if not plotter.x_data:
        plotter.get_logger().error("❌ No data received! Is the simulation running and robot moving?")
        return

    # Create the graph figure
    plt.figure(figsize=(12, 5))
    
    # Left Graph: Robot Path
    plt.subplot(1, 2, 1)
    plt.plot(plotter.x_data, plotter.y_data, '-b', linewidth=2)
    plt.title('Robot Trajectory (Bezier Square)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.grid(True)
    plt.axis('equal')
    
    # Right Graph: Velocity
    plt.subplot(1, 2, 2)
    min_len = min(len(plotter.times), len(plotter.v_data))
    plt.plot(plotter.times[:min_len], plotter.v_data[:min_len], '-r', linewidth=2)
    plt.title('Linear Velocity Profile')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    
    plt.tight_layout()
    
    # Save the file directly to your Desktop!
    desktop_path = os.path.expanduser('~/Desktop/bezier_trajectory_plot.png')
    plt.savefig(desktop_path, dpi=300)
    
    print("\n=======================================================")
    print(f"✅ SUCCESS! Plot saved directly to your Desktop!")
    print(f"📍 Location: {desktop_path}")
    print("=======================================================\n")
    
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
