#include <limits>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "assignment2/srv/set_threshold.hpp"
#include "assignment2/srv/get_vel_avg.hpp"

/**
 * User interface node for manual robot teleoperation
 *
 * This node provides a simple terminal-based interface that allows the user
 * to send velocity commands to the robot and interact with the services
 * exposed by the safety node (threshold update and velocity averaging)
 */

class UserInterfaceNode : public rclcpp::Node {
public:
    UserInterfaceNode() : Node("user_interface_node") {

        // Publisher for raw velocity commands (before safety filtering)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);

        // Service clients for interacting with the safety node
        client_set_threshold_ = this->create_client<assignment2::srv::SetThreshold>("set_threshold");
        client_get_avg_ = this->create_client<assignment2::srv::GetVelAvg>("get_vel_avg");

        // Display the control menu and start the input loop
        print_menu();
        input_loop();
    }

private:

    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    // Service clients
    rclcpp::Client<assignment2::srv::SetThreshold>::SharedPtr client_set_threshold_;
    rclcpp::Client<assignment2::srv::GetVelAvg>::SharedPtr client_get_avg_;

    // Default speed parameters
    double linear_speed_ = 0.5;
    double angular_speed_ = 1.0;

    // Prints the available commands to the terminal
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

    /**
     * Main input loop that waits for user commands
     *
     * This loop runs until ROS is shut down or the user presses 'q'
     */
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

    /**
     * Publishes a velocity command to the robot
     *
     * Input:
     *   lin = Linear velocity 
     *   ang = Angular velocity 
     */
    void publish_velocity(double lin, double ang) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = lin;
        msg.angular.z = ang;
        vel_pub_->publish(msg);

        std::cout << "Published velocity -> linear: " << lin
                  << ", angular: " << ang << "\n";
    }

    // Allows the user to update the linear speed parameter
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

    // Allows the user to update the angular speed parameter
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

    // Sends a request to update the safety threshold
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

    // Calls the service that returns the average of the last 5 velocity commands
    void call_get_avg() {
        if (!client_get_avg_->wait_for_service(std::chrono::seconds(1))) {
            std::cout << "Service get_vel_avg not available.\n";
            return;
        }

        auto req = std::make_shared<assignment2::srv::GetVelAvg::Request>();

        auto future = client_get_avg_->async_send_request(req);

        // Wait for the service response
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