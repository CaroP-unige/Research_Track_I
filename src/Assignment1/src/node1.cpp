#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <chrono>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

class TurtlesimUI : public rclcpp::Node {
public:
    TurtlesimUI() : Node("node1") {
        // Publisher per le due turtles
        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        // Creazione turtle2
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 2.0;
        request->y = 1.0;
        request->theta = 0.0;
        request->name = "turtle2";

        auto result = client->async_send_request(request);

        // Timer per gestire l'interazione utente
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

        std::cout << "\nWhich turtle do you want to move? (1=turtle1, 2=turtle2, 0=exit): ";
        std::cin >> turtle_choice;
        if (turtle_choice == 0) {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
            return;
        }
        if (turtle_choice != 1 && turtle_choice != 2) {
            std::cout << "Invalid turtle selection!\n";
            return;
        }

        std::cout << "Direction? (1=right, 2=left, 3=forward, 4=backward): ";
        std::cin >> direction;
        if (direction < 1 || direction > 4) {
            std::cout << "Invalid direction!\n";
            return;
        }

        std::cout << "Speed (>0): ";
        std::cin >> speed;
        if (speed <= 0) {
            std::cout << "Speed must be positive!\n";
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

        // Timer per fermare la turtle dopo 1 secondo
        auto stop_timer = this->create_wall_timer(
            1s,
            [publisher, this]() {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
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
