#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

enum State {FOLLOWING, FINISHED};

class PathController : public rclcpp::Node {
public:
    PathController() : Node("controller_node"), current_state_(FOLLOWING) {
        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/smooth_path", 10, std::bind(&PathController::traj_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathController::odom_callback, this, std::placeholders::_1));
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&PathController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Controller Ready. State: FOLLOWING");
    }

private:
    void traj_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_traj_ = *msg;
        has_traj_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p;
        m.getRPY(r, p, robot_yaw_);
    }

    void control_loop() {
        auto cmd = geometry_msgs::msg::Twist();

        if (!has_traj_ || current_traj_.poses.empty() || current_state_ == FINISHED) {
            if (current_state_ == FINISHED) vel_pub_->publish(cmd); 
            return;
        }

        // --- THE BUG FIX: Memory Flag ---
        // If we move 1 meter away, remember it permanently.
        if (std::sqrt(robot_x_*robot_x_ + robot_y_*robot_y_) > 1.0) {
            has_left_start_ = true; 
        }

        double dist_to_final = std::sqrt(
            std::pow(current_traj_.poses.back().pose.position.x - robot_x_, 2) + 
            std::pow(current_traj_.poses.back().pose.position.y - robot_y_, 2));

        // Now we only check if we are near the end, AND we remember leaving the start
        if (dist_to_final < 0.4 && has_left_start_) {
            current_state_ = FINISHED;
            RCLCPP_INFO(this->get_logger(), "--- GOAL REACHED! LOCKING MOTORS ---");
            vel_pub_->publish(cmd); 
            return;
        }

        size_t closest_idx = 0;
        double min_dist = 1e6;
        for (size_t i = 0; i < current_traj_.poses.size(); ++i) {
            double dx = current_traj_.poses[i].pose.position.x - robot_x_;
            double dy = current_traj_.poses[i].pose.position.y - robot_y_;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < min_dist) { min_dist = d; closest_idx = i; }
        }

        size_t target_idx = std::min(closest_idx + 10, current_traj_.poses.size() - 1);
        auto target = current_traj_.poses[target_idx].pose.position;
        double error_yaw = std::atan2(target.y - robot_y_, target.x - robot_x_) - robot_yaw_;
        
        while (error_yaw > M_PI) error_yaw -= 2*M_PI;
        while (error_yaw < -M_PI) error_yaw += 2*M_PI;

        cmd.linear.x = (std::abs(error_yaw) > 0.4) ? 0.05 : 0.15;
        cmd.angular.z = 1.0 * error_yaw;

        vel_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path current_traj_;
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    bool has_traj_ = false;
    bool has_left_start_ = false; // THE NEW FLAG
    State current_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathController>());
    rclcpp::shutdown();
    return 0;
}
