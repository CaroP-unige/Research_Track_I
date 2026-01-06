#include <deque>
#include <limits>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "assignment2/srv/get_vel_avg.hpp"
#include "assignment2/msg/obstacle_info.hpp"
#include "assignment2/srv/set_threshold.hpp"

/**
 * Safety node responsible for obstacle detection, safety filtering,
 * rollback behavior, and service-based interaction
 *
 * This node receives raw velocity commands from the user interface,
 * processes LaserScan data to detect obstacles, and enforces safety rules
 * It publishes filtered velocity commands, provides obstacle information,
 * and exposes services to adjust the safety threshold and compute velocity averages
 */

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {

        // Subscriber for raw velocity commands (before safety filtering)
        cmd_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_raw",
            10,
            std::bind(&SafetyNode::cmdRawCallback, this, std::placeholders::_1)
        );

        // Publisher for final velocity commands (after safety logic)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber for LaserScan data used for obstacle detection
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            std::bind(&SafetyNode::scanCallback, this, std::placeholders::_1)
        );

        // Publisher for obstacle information (10 Hz)
        obstacle_pub_ = this->create_publisher<assignment2::msg::ObstacleInfo>(
            "obstacle_info", 10
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SafetyNode::publishObstacleInfo, this)
        );

        // Service to update the safety threshold
        set_threshold_srv_ = this->create_service<assignment2::srv::SetThreshold>(
            "set_threshold",
            std::bind(&SafetyNode::setThresholdCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Service to compute the average of the last 5 velocity commands
        get_vel_avg_srv_ = this->create_service<assignment2::srv::GetVelAvg>(
            "get_vel_avg",
            std::bind(&SafetyNode::getVelAvgCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "SafetyNode (Step 3 completo) avviato");
    }

private:
    
    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Obstacle info publisher and timer
    rclcpp::Publisher<assignment2::msg::ObstacleInfo>::SharedPtr obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Services
    rclcpp::Service<assignment2::srv::SetThreshold>::SharedPtr set_threshold_srv_;
    rclcpp::Service<assignment2::srv::GetVelAvg>::SharedPtr get_vel_avg_srv_;

    // Laser scan state
    double min_distance_ = std::numeric_limits<double>::infinity();
    std::string direction_ = "none";

    // Safety threshold 
    double threshold_ = 0.5;

    // Last received raw velocity command
    geometry_msgs::msg::Twist last_raw_cmd_;

    // Queue storing the last 5 velocity commands
    std::deque<geometry_msgs::msg::Twist> last_cmds_;

    // Rollback state flag
    bool rollback_active_ = false;

    /**
     * Callback for raw velocity commands
     *
     * Stores the command, updates the rolling queue, and triggers safety logic
     */
    void cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_raw_cmd_ = *msg;

       // Maintain a queue of the last 5 commands
        last_cmds_.push_back(*msg);
        if (last_cmds_.size() > 5)
            last_cmds_.pop_front();

        applySafetyLogic();
    }

    /**
     * Callback for LaserScan messages
     *
     * Divides the scan into left, front, and right sectors and determines
     * the minimum distance and direction of the closest obstacle
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        const auto & ranges = msg->ranges;
        int n = ranges.size();
        if (n == 0) return;

        // Divide scan into three equal sectors
        int left_start = 0;
        int left_end   = n / 3;

        int front_start = n / 3;
        int front_end   = 2 * n / 3;

        int right_start = 2 * n / 3;
        int right_end   = n;

        // Compute minimum distances in each sector
        double min_left  = findMin(ranges, left_start, left_end);
        double min_front = findMin(ranges, front_start, front_end);
        double min_right = findMin(ranges, right_start, right_end);

        // Determine global minimum and direction
        min_distance_ = std::min({min_left, min_front, min_right});

        if (min_distance_ == min_front)
            direction_ = "front";
        else if (min_distance_ == min_left)
            direction_ = "left";
        else
            direction_ = "right";

        applySafetyLogic();
    }

    // Utility function to compute the minimum valid distance in a sector
    double findMin(const std::vector<float> & ranges, int start, int end) {
        double m = std::numeric_limits<double>::infinity();
        for (int i = start; i < end; i++) {
            if (std::isfinite(ranges[i]) && ranges[i] > 0.01) {
                m = std::min(m, static_cast<double>(ranges[i]));
            }
        }
        return m;
    }

    /**
     * Core safety logic implementing obstacle avoidance and rollback behavior
     *
     * - If an obstacle is too close, rollback is triggered
     * - Rollback overrides user commands until the robot is safe again
     * - Rotational commands are allowed even when obstacles are close
     */
    void applySafetyLogic() {

        bool going_forward  = last_raw_cmd_.linear.x > 0.0;
        bool going_backward = last_raw_cmd_.linear.x < 0.0;
        bool rotating       = last_raw_cmd_.angular.z != 0.0;

        bool obstacle_too_close = (min_distance_ < threshold_);

        // Trigger rollback if needed
        if (!rollback_active_ && obstacle_too_close) {

            // Allow rotation even when obstacles are close
            if (rotating) {
                cmd_pub_->publish(last_raw_cmd_);
                return;
            }

            rollback_active_ = true;

            if (going_forward)
                RCLCPP_WARN(this->get_logger(), "Rollback attivato (stavi andando AVANTI)");
            else if (going_backward)
                RCLCPP_WARN(this->get_logger(), "Rollback attivato (stavi andando INDIETRO)");
        }

        // Rollback behavior
        if (rollback_active_) {

            geometry_msgs::msg::Twist rb_cmd;

            // Reverse direction depending on last command
            if (last_raw_cmd_.linear.x > 0.0)
                rb_cmd.linear.x = -0.2;
            else if (last_raw_cmd_.linear.x < 0.0)
                rb_cmd.linear.x = 0.2;

            rb_cmd.angular.z = 0.0;

            cmd_pub_->publish(rb_cmd);

            // Stop rollback once safe
            if (min_distance_ > threshold_) {
                rollback_active_ = false;

                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;

                cmd_pub_->publish(stop_cmd);
                last_raw_cmd_ = stop_cmd;

                RCLCPP_INFO(this->get_logger(), "Rollback completato. Robot fermato.");
            }

            return;
        }

        // Normal behavior: forward raw command
        cmd_pub_->publish(last_raw_cmd_);
    }

    // Publishes obstacle information at 10 Hz
    void publishObstacleInfo() {
        assignment2::msg::ObstacleInfo msg;
        msg.min_distance = min_distance_;
        msg.direction = direction_;
        obstacle_pub_->publish(msg);
    }

    // Service callback to update the safety threshold
    void setThresholdCallback(
        const std::shared_ptr<assignment2::srv::SetThreshold::Request> req,
        std::shared_ptr<assignment2::srv::SetThreshold::Response> res)
    {
        threshold_ = req->threshold;
        res->success = true;
        res->message = "Nuova soglia impostata correttamente";

        RCLCPP_INFO(this->get_logger(),
            "Nuova soglia di sicurezza: %.2f", threshold_);
    }

    // Service callback to compute the average of the last 5 velocity commands
    void getVelAvgCallback(
        const std::shared_ptr<assignment2::srv::GetVelAvg::Request>,
        std::shared_ptr<assignment2::srv::GetVelAvg::Response> res)
    {
        if (last_cmds_.empty()) {
            res->avg_linear = 0.0;
            res->avg_angular = 0.0;
            return;
        }

        double sum_lin = 0.0;
        double sum_ang = 0.0;

        for (const auto & cmd : last_cmds_) {
            sum_lin += cmd.linear.x;
            sum_ang += cmd.angular.z;
        }

        res->avg_linear = sum_lin / last_cmds_.size();
        res->avg_angular = sum_ang / last_cmds_.size();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}