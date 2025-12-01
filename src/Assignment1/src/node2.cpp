/*
* ROS2 node responsible for monitoring the distance between turtle1 and turtle2 and handling corrections in case they get too close
* or violate the boundaries of the Turtlesim window.
*/

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class DistanceMonitor : public rclcpp::Node {
public:
    DistanceMonitor() : Node("node2") {

        // Subscriber to receive the position of turtle1
        sub1_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&DistanceMonitor::pose_callback1, this, std::placeholders::_1)
        );

        // Subscriber to receive the position of turtle2
        sub2_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10,
            std::bind(&DistanceMonitor::pose_callback2, this, std::placeholders::_1)
        );

        // Publisher to publish the distance calculated between the two turtles
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        // Publisher to send stop or correction commands to turtle1 and turtle2
        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        // Periodic timer to check distance and boundaries
        timer_ = this->create_wall_timer(100ms, std::bind(&DistanceMonitor::check_distance, this));
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1_; 
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub2_; 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose pose1_; // Current position of turtle1
    turtlesim::msg::Pose pose2_; // Current position of turtle2
    bool pose1_ready_ = false; // Flag: true if turtle1's position has been received at least once
    bool pose2_ready_ = false; // Flag: true if turtle2's position has been received at least once

    const float distance_threshold_ = 1.0; // minimum allowed distance between the turtles
    const float x_min_ = 1.0, x_max_ = 10.0; // X limits of the workspace
    const float y_min_ = 1.0, y_max_ = 10.0; // Y limits of the workspace

    // Callback to update the position of turtle1
    void pose_callback1(const turtlesim::msg::Pose::SharedPtr msg) {
        pose1_ = *msg;
        pose1_ready_ = true;
        // Border check for turtle1
        check_boundaries(pose1_, pub1_, "turtle1");
    }

    // Callback to update the position of turtle2
    void pose_callback2(const turtlesim::msg::Pose::SharedPtr msg) {
        pose2_ = *msg;
        pose2_ready_ = true;
        // Border check for turtle2
        check_boundaries(pose2_, pub2_, "turtle2");
    }

    // Function to check the position relative to the edges of the Turtlesim window
    void check_boundaries(const turtlesim::msg::Pose &pose,
                          rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
                          const std::string &name)
    {
        geometry_msgs::msg::Twist vel; // speed message to correct the position
        bool violation = false;

        // Check upper and lower limit in X
        if (pose.x > x_max_) { vel.linear.x = -0.5; violation = true; }
        else if (pose.x < x_min_) { vel.linear.x = 0.5; violation = true; }

        // Check upper and lower limit in Y
        if (pose.y > y_max_) { vel.linear.x = -0.5; violation = true; }
        else if (pose.y < y_min_) { vel.linear.x = 0.5; violation = true; }

        // If the turtle has exceeded the limits, send corrective command
        if (violation) {
            RCLCPP_WARN(this->get_logger(), "%s is OUTSIDE the window! Moving back...", name.c_str());
            pub->publish(vel);
            rclcpp::sleep_for(500ms); // Waits 0.5 seconds to allow the correction

            // Stop the turtle after the correction
            vel.linear.x = 0;
            pub->publish(vel);
        }
    }

    // Function to check the distance between the two turtles
    void check_distance() {

        // Check that both positions are available
        if (!pose1_ready_ || !pose2_ready_) return;

        // Euclidean distance calculation
        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Publish the distance on the dedicated topic
        std_msgs::msg::Float32 msg;
        msg.data = dist;
        dist_pub_->publish(msg);

        // Informational log of the current location and distance
        RCLCPP_INFO(this->get_logger(),
                    "Turtle1(x=%.2f, y=%.2f)  Turtle2(x=%.2f, y=%.2f)  Distance=%.2f",
                    pose1_.x, pose1_.y, pose2_.x, pose2_.y, dist);

        // If the turtles are too close together, move them slightly to separate them
        if (dist < distance_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Turtles TOO CLOSE! Moving them apart...");

            geometry_msgs::msg::Twist v1, v2;

            v1.linear.x = (pose1_.x > pose2_.x) ? 0.5 : -0.5;
            v2.linear.x = (pose2_.x > pose1_.x) ? 0.5 : -0.5;

            // Publish corrective commands
            pub1_->publish(v1);
            pub2_->publish(v2);

            rclcpp::sleep_for(500ms); // Waits 0.5 seconds to allow the correction

            // Stop both turtles after the corrective movement
            v1.linear.x = 0;
            v2.linear.x = 0;

            pub1_->publish(v1);
            pub2_->publish(v2);

            RCLCPP_WARN(this->get_logger(), "Corrective action completed.");
        }
    }
};

/*
* Main function: initializes ROS2, starts the node, and handles spinning.
*/

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
