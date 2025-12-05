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
    /*
    * Node constructor.
    *
    * Initializes subscribers for turtle poses and last user velocity commands,
    * publishers for distance and corrective commands, and a periodic timer.
    */
    DistanceMonitor() : Node("node2") {

        // Subscribers to receive the position of turtle1 and turtle2.
        sub1_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&DistanceMonitor::pose_callback1, this, std::placeholders::_1)
        );

        sub2_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10,
            std::bind(&DistanceMonitor::pose_callback2, this, std::placeholders::_1)
        );

        // Subscribers to receive the last user velocity commands.
        sub_vel1_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                prec_vel1_ = *msg;
            });

        sub_vel2_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "turtle2/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                prec_vel2_ = *msg;
            });

        // Publisher to publish the distance calculated between the two turtles.
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        // Publishers to send stop or corrective commands to turtle1 and turtle2.
        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        // Timer to periodically check distance and boundaries.
        timer_ = this->create_wall_timer(100ms, std::bind(&DistanceMonitor::check_distance, this));
    }

private:
    // Subscribers.
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1_; 
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub2_; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel1_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel2_;

    // Publishers.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Last received velocity commands from the user.
    geometry_msgs::msg::Twist prec_vel1_, prec_vel2_;

    // Current positions of the turtles.
    turtlesim::msg::Pose pose1_;
    turtlesim::msg::Pose pose2_;
    bool pose1_ready_ = false;
    bool pose2_ready_ = false;

    // Minimum allowed distance between the turtles.
    const float distance_threshold_ = 1.0;

    // Workspace boundaries.
    const float x_min_ = 1.0, x_max_ = 10.0;
    const float y_min_ = 1.0, y_max_ = 10.0;

    // Callback to update the position of turtle1, calls the boundary check function.
    void pose_callback1(const turtlesim::msg::Pose::SharedPtr msg) {
        pose1_ = *msg;
        pose1_ready_ = true;
        check_boundaries(pose1_, prec_vel1_, pub1_, "turtle1");
    }

    // Callback to update the position of turtle2, calls the boundary check function.
    void pose_callback2(const turtlesim::msg::Pose::SharedPtr msg) {
        pose2_ = *msg;
        pose2_ready_ = true;
        check_boundaries(pose2_, prec_vel2_, pub2_, "turtle2");
    }

    /*
    * Function to check if the turtle has exceeded the workspace boundaries
    * if so, send a corrective command using the last user velocity.
    */
    void check_boundaries(const turtlesim::msg::Pose &pose,
                          const geometry_msgs::msg::Twist &prec_vel,
                          rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
                          const std::string &name)
    {
        geometry_msgs::msg::Twist invert = prec_vel;

        if (pose.x < x_min_ || pose.x > x_max_ || pose.y < y_min_ || pose.y > y_max_) {

            // Create command by reversing the user's last speed.
            geometry_msgs::msg::Twist invert = prec_vel;
            invert.linear.x = -prec_vel.linear.x;

            RCLCPP_WARN(this->get_logger(), "%s is OUTSIDE the window! Moving back...", name.c_str());
            pub->publish(invert);

            rclcpp::sleep_for(200ms); // Waits half a second for the correction

            // Turtle stop after adjustment.
            invert.linear.x = 0.0;
            pub->publish(invert);
        }
    }

    /*
    * Function to monitor the distance between the two turtles,
    * publish the current distance, and apply corrective actions 
    * if they get too close to each other.
    */
    void check_distance() {

        // Check that both positions are available.
        if (!pose1_ready_ || !pose2_ready_) return;

        // Euclidean distance calculation.
        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Publish the distance on the dedicated topic.
        std_msgs::msg::Float32 msg;
        msg.data = dist;
        dist_pub_->publish(msg);

        // Informational log of the current location and distance.
        RCLCPP_INFO(this->get_logger(),
                    "Turtle1(x=%.2f, y=%.2f)  Turtle2(x=%.2f, y=%.2f)  Distance=%.2f",
                    pose1_.x, pose1_.y, pose2_.x, pose2_.y, dist);

        // If the turtles are too close together, move them slightly to separate them.
        if (dist < distance_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Turtles TOO CLOSE! Moving them apart...");

            // Normalized direction from turtle1 to turtle2
            float nx = dx / dist;
            float ny = dy / dist;

            // Convert each turtle's velocity from body frame → world frame
            float v1x = prec_vel1_.linear.x * std::cos(pose1_.theta);
            float v1y = prec_vel1_.linear.x * std::sin(pose1_.theta);

            float v2x = prec_vel2_.linear.x * std::cos(pose2_.theta);
            float v2y = prec_vel2_.linear.x * std::sin(pose2_.theta);

            // Dot products: tells if they are approaching each other
            // dot1 < 0 → turtle1 is moving toward turtle2
            // dot2 < 0 → turtle2 is moving toward turtle1
            float dot1 = v1x * nx + v1y * ny;
            float dot2 = v2x * (-nx) + v2y * (-ny);

            // Prepare stop message
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0;

            // Turtles rebound ONLY if they are approaching
            if (dot1 < 0) {
                geometry_msgs::msg::Twist invert = prec_vel1_;
                invert.linear.x = -prec_vel1_.linear.x;

                RCLCPP_WARN(this->get_logger(), " → Turtle1 is approaching! Reversing it.");
                pub1_->publish(invert);
                rclcpp::sleep_for(300ms);
                pub1_->publish(stop);
            }

            else if (dot2 < 0) {
                geometry_msgs::msg::Twist invert = prec_vel2_;
                invert.linear.x = -prec_vel2_.linear.x;

                RCLCPP_WARN(this->get_logger(), " → Turtle2 is approaching! Reversing it.");
                pub2_->publish(invert);
                rclcpp::sleep_for(300ms);
                pub2_->publish(stop);
            }
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