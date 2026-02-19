#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class TrajectoryGenerator : public rclcpp::Node {
public:
    TrajectoryGenerator() : Node("trajectory_generator_node") {
        // Subscribe to the smooth path we just made
        subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/smooth_path", 10, std::bind(&TrajectoryGenerator::path_callback, this, std::placeholders::_1));
        
        // Publish the trajectory (using Path msg for visualization, but conceptually it has time)
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
        
        RCLCPP_INFO(this->get_logger(), "Trajectory Generator Waiting for Path...");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    
    // Robot Constraints
    const double MAX_VEL = 0.22; // m/s (Turtlebot limit)
    const double ACCEL = 0.1;    // m/s^2

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        auto traj_msg = nav_msgs::msg::Path();
        traj_msg.header = msg->header;
        
        double current_time = 0.0;
        double current_vel = 0.0;
        
        // Loop through points to assign time based on distance
        for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
            auto p1 = msg->poses[i].pose.position;
            auto p2 = msg->poses[i+1].pose.position;
            
            // Calculate distance between points
            double dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
            
            // Simple Trapezoidal Profile Logic:
            // 1. Ramp up velocity if we are far from end
            // 2. Cap at MAX_VEL
            // 3. Ramp down if close to end (simplified)
            
            if (current_vel < MAX_VEL) {
                current_vel += ACCEL * 0.1; // Assume 0.1s time step for simplicity
            }
            
            // Time to travel this segment = distance / velocity
            double dt = (current_vel > 0) ? (dist / current_vel) : 0.1;
            current_time += dt;
            
            // We store the timestamp in the header of each pose (sec + nanosec)
            traj_msg.poses.push_back(msg->poses[i]);
            traj_msg.poses.back().header.stamp.sec = (int)current_time;
            traj_msg.poses.back().header.stamp.nanosec = (current_time - (int)current_time) * 1e9;
        }
        
        publisher_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Generated Trajectory with %lu points", traj_msg.poses.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}
