#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

using namespace std::chrono_literals;

class DistanceMonitor : public rclcpp::Node {
public:
    DistanceMonitor() : Node("node2") {
        // Subscribers per le pose delle tartarughe
        sub1_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&DistanceMonitor::pose_callback1, this, std::placeholders::_1)
        );
        sub2_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10, std::bind(&DistanceMonitor::pose_callback2, this, std::placeholders::_1)
        );

        // Publisher della distanza
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        // Publisher per fermare le tartarughe
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

    const float distance_threshold_ = 1.0;  // distanza minima
    const float x_min_ = 1.0, x_max_ = 10.0;
    const float y_min_ = 1.0, y_max_ = 10.0;

    void pose_callback1(const turtlesim::msg::Pose::SharedPtr msg) {
        pose1_ = *msg;
        pose1_ready_ = true;
    }

    void pose_callback2(const turtlesim::msg::Pose::SharedPtr msg) {
        pose2_ = *msg;
        pose2_ready_ = true;
    }

    void check_distance() {
        if (!pose1_ready_ || !pose2_ready_) return;

        // calcola distanza euclidea
        float dx = pose1_.x - pose2_.x;
        float dy = pose1_.y - pose2_.y;
        float dist = std::sqrt(dx*dx + dy*dy);

        // pubblica la distanza
        std_msgs::msg::Float32 msg;
        msg.data = dist;
        dist_pub_->publish(msg);

        // controlla la soglia distanza
        if (dist < distance_threshold_ ||
            pose1_.x < x_min_ || pose1_.x > x_max_ || pose1_.y < y_min_ || pose1_.y > y_max_ ||
            pose2_.x < x_min_ || pose2_.x > x_max_ || pose2_.y < y_min_ || pose2_.y > y_max_) 
        {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            pub1_->publish(stop_msg);
            pub2_->publish(stop_msg);
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
