/*
* ROS2 node for manual control of two turtles in Turtlesim.
*
* This node allows the user to select a turtle, choose a direction, specify a speed, and send a movement command. 
* The node publishes to turtle1/cmd_vel and turtle2/cmd_vel. Each movement lasts 1 second, after which a stop command is automatically sent.
*/

#include <chrono>
#include <memory>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using namespace std::chrono_literals;

class TurtlesimUI : public rclcpp::Node {
public:

    /*
    * Node constructor.
    *
    * Initializes the publishers for the two turtles and a timer that cyclically allows the user to enter commands.
    */

    TurtlesimUI() : Node("node1") {

        // Publisher of velocity commands for turtle1 and turtle2.
        pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        // Timer that repeatedly calls the input acquisition function.
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtlesimUI::user_input_loop, this));
    }

private:

    // Publisher for the speed commands of the two turtles.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub2_;

    // Timer for periodically reading user input.
    rclcpp::TimerBase::SharedPtr timer_;

    // Flag that prevents duplicate input requests during processing.
    bool waiting_for_input_ = true;

    /*
    * Keyboard input acquisition loop.
    *
    * Asks the user which turtle to move, the desired direction, and the speed. 
    * Checks the validity of the input and calls the function to send the command.
    */

    void user_input_loop() {
        if (!waiting_for_input_) return;

        int turtle_choice;
        int direction;
        float speed;

        // Selection of the turtle to control.
        cout << "\nWhich turtle do you want to move? (1=turtle1, 2=turtle2, 0=exit): ";
        cin >> turtle_choice;
        if(cin.fail()) {        
            cin.clear();         
            cin.ignore(10000,'\n'); 
            cout << "Invalid input! Please enter a number.\n";
            return;
        }
        if (turtle_choice == 0) {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
            return;
        }
        if (turtle_choice != 1 && turtle_choice != 2) {
            cout << "Invalid turtle selection!\n";
            return;
        }

        // Selection of the direction of movement.
        cout << "Direction? (1=rotate right, 2=rotate left, 3=move forward, 4=move backward): ";
        cin >> direction;
        if(cin.fail()) {        
            cin.clear();         
            cin.ignore(10000,'\n'); 
            cout << "Invalid input! Please enter a number.\n";
            return;
        }
        if (direction < 1 || direction > 4) {
            cout << "Invalid direction!\n";
            return;
        }

        // Selection of the speed.
        cout << "Speed (>0): ";
        cin >> speed;
        if(cin.fail()) {        
            cin.clear();         
            cin.ignore(10000,'\n'); 
            cout << "Invalid input! Please enter a number.\n";
            return;
        }
        if (speed <= 0) {
            cout << "Speed must be positive!\n";
            return;
        }

        // Sending the command to the selected turtle.
        move_turtle(turtle_choice, direction, speed);
    }

    /**
    * Function that sends a movement command to the selected turtle.
    *
    * Creates a Twist message based on the direction chosen by the user and publishes it to the corresponding topic. 
    * Then sets a timer that, after 1 second, automatically sends a stop command.
    *
    * Parameters: 
    *   'turtle_choice' = 1 for turtle1, 2 for turtle2
    *   'direction' = chosen direction (1-4)
    *   'speed' = intensity of linear or angular speed
    */

    void move_turtle(int turtle_choice, int direction, float speed) {
        geometry_msgs::msg::Twist msg;

        // Setting the command based on the direction.
        switch (direction) {
            case 1: msg.angular.z = -speed; break; // right rotation 
            case 2: msg.angular.z = speed; break;  // left rotation
            case 3: msg.linear.x = speed; break;   // move forward
            case 4: msg.linear.x = -speed; break;  // move backward
        }

        // Choosing the correct publisher based on the selected turtle.
        auto publisher = (turtle_choice == 1) ? pub1_ : pub2_;

        // Publication of the movement command.
        publisher->publish(msg);
    
        // Creation of a secondary timer whose sole function is to send a stop command after 1 second.
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

/*
* Main function: initializes ROS2, starts the node, and handles spinning.
*/

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlesimUI>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}