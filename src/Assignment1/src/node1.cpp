#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using namespace std::chrono_literals;

class TurtlesimUI : public rclcpp::Node {
public:
    TurtlesimUI() : Node("node1") {

        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&TurtlesimUI::user_input_loop, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool waiting_for_input_ = true;

    void user_input_loop() {
        if (!waiting_for_input_) return;

        int turtle_choice;
        int direction;
        float speed;

        cout << "\nWhich turtle do you want to move? (1=turtle1, 2=turtle2, 0=exit): ";
        cin >> turtle_choice;
        if (turtle_choice == 0) {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
            return;
        }
        if (turtle_choice != 1 && turtle_choice != 2) {
            cout << "Invalid turtle selection!\n";
            return;
        }

        cout << "Direction? (1=rotate right, 2=rotate left, 3=move forward, 4=move backward): ";
        cin >> direction;
        if (direction < 1 || direction > 4) {
            cout << "Invalid direction!\n";
            return;
        }

        cout << "Speed (>0): ";
        cin >> speed;
        if (speed <= 0) {
            cout << "Speed must be positive!\n";
            return;
        }

        move_turtle(turtle_choice, direction, speed);
    }

    void move_turtle(int turtle_choice, int direction, float speed) {
        geometry_msgs::msg::Twist msg;

        switch (direction) {
            case 1: msg.angular.z = -speed; break; // right 
            case 2: msg.angular.z = speed; break;  // left
            case 3: msg.linear.x = speed; break;   // forward
            case 4: msg.linear.x = -speed; break;  // backward
        }

        auto publisher = (turtle_choice == 1) ? pub1_ : pub2_;
        publisher->publish(msg);

        auto stop_timer = this->create_wall_timer(
            1s,
            [publisher, this]() {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.linear.y = 0.0;
                publisher->publish(stop_msg);
            }
        );
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimUI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}