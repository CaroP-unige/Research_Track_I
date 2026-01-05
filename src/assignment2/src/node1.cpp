#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignment2/srv/set_threshold.hpp"
#include "assignment2/srv/get_vel_avg.hpp"
#include <iostream>
#include <limits>

class UserInterfaceNode : public rclcpp::Node {
public:
    UserInterfaceNode() : Node("user_interface_node") {

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);

        client_set_threshold_ = this->create_client<assignment2::srv::SetThreshold>("set_threshold");
        client_get_avg_ = this->create_client<assignment2::srv::GetVelAvg>("get_vel_avg");

        print_menu();
        input_loop();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Client<assignment2::srv::SetThreshold>::SharedPtr client_set_threshold_;
    rclcpp::Client<assignment2::srv::GetVelAvg>::SharedPtr client_get_avg_;

    double linear_speed_ = 0.5;
    double angular_speed_ = 1.0;

    void print_menu() {
        std::cout << "\n=== Robot Teleop Interface ===\n";
        std::cout << "w - Forward\n";
        std::cout << "s - Backward\n";
        std::cout << "a - Rotate left\n";
        std::cout << "d - Rotate right\n";
        std::cout << "x - Stop\n";
        std::cout << "t - Set new threshold\n";
        std::cout << "m - Get averages of last 5 velocities\n";
        std::cout << "l - Set linear speed (current: " << linear_speed_ << ")\n";
        std::cout << "r - Set angular speed (current: " << angular_speed_ << ")\n";
        std::cout << "q - Quit\n";
        std::cout << "===============================\n\n";
    }

    void input_loop() {
        char c;

        while (rclcpp::ok()) {
            std::cout << "> ";
            std::cin >> c;

            if (c == 'q') {
                std::cout << "Exiting...\n";
                rclcpp::shutdown();
                break;
            }

            switch (c) {
                case 'w': publish_velocity(+linear_speed_, 0.0); break;
                case 's': publish_velocity(-linear_speed_, 0.0); break;
                case 'a': publish_velocity(0.0, +angular_speed_); break;
                case 'd': publish_velocity(0.0, -angular_speed_); break;
                case 'x': publish_velocity(0.0, 0.0); break;
                case 't': call_set_threshold(); break;
                case 'm': call_get_avg(); break;
                case 'l': set_linear_speed(); break;
                case 'r': set_angular_speed(); break;
                default:
                    std::cout << "Invalid command.\n";
                    break;
            }
        }
    }

    void publish_velocity(double lin, double ang) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = lin;
        msg.angular.z = ang;
        vel_pub_->publish(msg);

        std::cout << "Published velocity -> linear: " << lin
                  << ", angular: " << ang << "\n";
    }

    void set_linear_speed() {
        std::cout << "Enter new linear speed (> 0): ";
        double value;
        std::cin >> value;

        if (std::cin.fail() || value <= 0) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid speed. Must be a positive number.\n";
            return;
        }

        linear_speed_ = value;
        std::cout << "Linear speed updated to " << linear_speed_ << "\n";
    }

    void set_angular_speed() {
        std::cout << "Enter new angular speed (> 0): ";
        double value;
        std::cin >> value;

        if (std::cin.fail() || value <= 0) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid speed. Must be a positive number.\n";
            return;
        }

        angular_speed_ = value;
        std::cout << "Angular speed updated to " << angular_speed_ << "\n";
    }

    void call_set_threshold() {
        if (!client_set_threshold_->wait_for_service(std::chrono::seconds(1))) {
            std::cout << "Service set_threshold not available.\n";
            return;
        }

        double new_thr;
        std::cout << "Enter new threshold: ";
        std::cin >> new_thr;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid threshold.\n";
            return;
        }

        auto req = std::make_shared<assignment2::srv::SetThreshold::Request>();
        req->threshold = new_thr;

        client_set_threshold_->async_send_request(req);

        std::cout << "Threshold update request sent.\n";
    }

    void call_get_avg() {
        if (!client_get_avg_->wait_for_service(std::chrono::seconds(1))) {
            std::cout << "Service get_vel_avg not available.\n";
            return;
        }

        auto req = std::make_shared<assignment2::srv::GetVelAvg::Request>();

        auto future = client_get_avg_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS) {

            auto res = future.get();
            std::cout << "Average linear velocity: " << res->avg_linear << "\n";
            std::cout << "Average angular velocity: " << res->avg_angular << "\n";
        } else {
            std::cout << "Failed to call get_vel_avg service.\n";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
