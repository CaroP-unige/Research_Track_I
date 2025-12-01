#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

using namespace std::chrono_literals;

class DistanceMonitor : public rclcpp::Node {
public:
    DistanceMonitor() : Node("node2") {

        sub1_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&DistanceMonitor::pose_callback1, this, std::placeholders::_1)
        );

        sub2_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10,
            std::bind(&DistanceMonitor::pose_callback2, this, std::placeholders::_1)
        );

        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&DistanceMonitor::check_distance, this));
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;

    turtlesim::msg::Pose pose1_;
    turtlesim::msg::Pose pose2_;
    bool pose1_ready_ = false;
    bool pose2_ready_ = false;

    const float distance_threshold_ = 1.0;
    const float x_min_ = 1.0, x_max_ = 10.0;
    const float y_min_ = 1.0, y_max_ = 10.0;

    void pose_callback1(const turtlesim::msg::Pose::SharedPtr msg) {
        pose1_ = *msg;
        pose1_ready_ = true;
        check_boundaries(pose1_, pub1_, "turtle1");
    }

    void pose_callback2(const turtlesim::msg::Pose::SharedPtr msg) {
        pose2_ = *msg;
        pose2_ready_ = true;
        check_boundaries(pose2_, pub2_, "turtle2");
    }

    void check_boundaries(const turtlesim::msg::Pose &pose,
                          rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
                          const std::string &name)
    {
        geometry_msgs::msg::Twist vel;
        bool violation = false;

        if (pose.x > x_max_) { vel.linear.x = -0.5; violation = true; }
        else if (pose.x < x_min_) { vel.linear.x = 0.5; violation = true; }

        if (pose.y > y_max_) { vel.linear.y = -0.5; violation = true; }
        else if (pose.y < y_min_) { vel.linear.y = 0.5; violation = true; }

        if (violation) {
            RCLCPP_WARN(this->get_logger(), "%s is OUTSIDE the window! Moving back...", name.c_str());
            pub->publish(vel);
            rclcpp::sleep_for(500ms);

            vel.linear.x = 0;
            vel.linear.y = 0;
            pub->publish(vel);
        }
    }

    void check_distance() {
        if (!pose1_ready_ || !pose2_ready_) return;

        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        std_msgs::msg::Float32 msg;
        msg.data = dist;
        dist_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "Turtle1(x=%.2f, y=%.2f)  Turtle2(x=%.2f, y=%.2f)  Distance=%.2f",
                    pose1_.x, pose1_.y, pose2_.x, pose2_.y, dist);

        if (dist < distance_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Turtles TOO CLOSE! Moving them apart...");

            geometry_msgs::msg::Twist v1, v2;

            v1.linear.x = (pose1_.x > pose2_.x) ? 0.5 : -0.5;
            v1.linear.y = (pose1_.y > pose2_.y) ? 0.5 : -0.5;

            v2.linear.x = (pose2_.x > pose1_.x) ? 0.5 : -0.5;
            v2.linear.y = (pose2_.y > pose1_.y) ? 0.5 : -0.5;

            pub1_->publish(v1);
            pub2_->publish(v2);

            rclcpp::sleep_for(500ms);

            v1.linear.x = v1.linear.y = 0;
            v2.linear.x = v2.linear.y = 0;

            pub1_->publish(v1);
            pub2_->publish(v2);

            RCLCPP_WARN(this->get_logger(), "Corrective action completed.");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
