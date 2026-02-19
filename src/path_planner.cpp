#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

struct Point {
    double x, y;
};

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner_node") {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&PathPlanner::publish_path, this));

        // Your Square Waypoints
        raw_waypoints_ = {
            {0.0, 0.0}, 
            {2.0, 0.0}, 
            {2.0, 2.0}, 
            {0.0, 2.0}, 
            {0.0, 0.0}
        };
        
        RCLCPP_INFO(this->get_logger(), "Path Planner: Generating Smooth Bezier Square...");
    }

private:
    std::vector<Point> raw_waypoints_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Quadratic Bezier: B(t) = (1-t)^2*P0 + 2(1-t)t*P1 + t^2*P2
    Point get_bezier_point(float t, Point p0, Point p1, Point p2) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;
        Point p;
        p.x = uu * p0.x + 2 * u * t * p1.x + tt * p2.x;
        p.y = uu * p0.y + 2 * u * t * p1.y + tt * p2.y;
        return p;
    }

    void publish_path() {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "odom"; // MATCH WITH CONTROLLER

        for (size_t i = 0; i < raw_waypoints_.size() - 1; ++i) {
            Point p0 = raw_waypoints_[i];     
            Point p2 = raw_waypoints_[i+1];   
            
            // Your Control Point Logic
            Point p1 = {(p0.x + p2.x) / 2.0 + 0.2, (p0.y + p2.y) / 2.0 + 0.2}; 

            for (float t = 0; t <= 1.0; t += 0.05) {
                Point smooth_pt = get_bezier_point(t, p0, p1, p2);
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = smooth_pt.x;
                pose.pose.position.y = smooth_pt.y;
                pose.pose.orientation.w = 1.0;
                
                path_msg.poses.push_back(pose);
            }
        }
        publisher_->publish(path_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
