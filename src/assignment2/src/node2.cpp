#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "assignment2/msg/obstacle_info.hpp"
#include "assignment2/srv/set_threshold.hpp"
#include "assignment2/srv/get_vel_avg.hpp"

#include <limits>
#include <string>
#include <deque>

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {

        // Subscriber ai comandi grezzi
        cmd_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_raw",
            10,
            std::bind(&SafetyNode::cmdRawCallback, this, std::placeholders::_1)
        );

        // Publisher dei comandi finali
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber al LaserScan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            std::bind(&SafetyNode::scanCallback, this, std::placeholders::_1)
        );

        // Publisher ObstacleInfo (10 Hz)
        obstacle_pub_ = this->create_publisher<assignment2::msg::ObstacleInfo>(
            "obstacle_info", 10
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SafetyNode::publishObstacleInfo, this)
        );

        // Service set_threshold
        set_threshold_srv_ = this->create_service<assignment2::srv::SetThreshold>(
            "set_threshold",
            std::bind(&SafetyNode::setThresholdCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Service get_vel_avg
        get_vel_avg_srv_ = this->create_service<assignment2::srv::GetVelAvg>(
            "get_vel_avg",
            std::bind(&SafetyNode::getVelAvgCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "SafetyNode (Step 3 completo) avviato");
    }

private:
    // Subscriber e publisher
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publisher ObstacleInfo
    rclcpp::Publisher<assignment2::msg::ObstacleInfo>::SharedPtr obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Services
    rclcpp::Service<assignment2::srv::SetThreshold>::SharedPtr set_threshold_srv_;
    rclcpp::Service<assignment2::srv::GetVelAvg>::SharedPtr get_vel_avg_srv_;

    // Stato del laser
    double min_distance_ = std::numeric_limits<double>::infinity();
    std::string direction_ = "none";

    // Soglia di sicurezza
    double threshold_ = 0.5;

    // Ultimo comando grezzo
    geometry_msgs::msg::Twist last_raw_cmd_;

    // Coda ultimi 5 comandi
    std::deque<geometry_msgs::msg::Twist> last_cmds_;

    // Stato rollback
    bool rollback_active_ = false;

    // -----------------------------
    // CALLBACK COMANDI GREZZI
    // -----------------------------
    void cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_raw_cmd_ = *msg;

        // Aggiorna coda ultimi 5 comandi
        last_cmds_.push_back(*msg);
        if (last_cmds_.size() > 5)
            last_cmds_.pop_front();

        applySafetyLogic();
    }

    // -----------------------------
    // CALLBACK LASER
    // -----------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        const auto & ranges = msg->ranges;
        int n = ranges.size();
        if (n == 0) return;

        int left_start = 0;
        int left_end   = n / 3;

        int front_start = n / 3;
        int front_end   = 2 * n / 3;

        int right_start = 2 * n / 3;
        int right_end   = n;

        double min_left  = findMin(ranges, left_start, left_end);
        double min_front = findMin(ranges, front_start, front_end);
        double min_right = findMin(ranges, right_start, right_end);

        min_distance_ = std::min({min_left, min_front, min_right});

        if (min_distance_ == min_front)
            direction_ = "front";
        else if (min_distance_ == min_left)
            direction_ = "left";
        else
            direction_ = "right";

        applySafetyLogic();
    }

    // -----------------------------
    // FUNZIONE UTILE: MINIMO LASER
    // -----------------------------
    double findMin(const std::vector<float> & ranges, int start, int end) {
        double m = std::numeric_limits<double>::infinity();
        for (int i = start; i < end; i++) {
            if (std::isfinite(ranges[i]) && ranges[i] > 0.01) {
                m = std::min(m, static_cast<double>(ranges[i]));
            }
        }
        return m;
    }

    // -----------------------------
    // LOGICA DI SICUREZZA + ROLLBACK
    // -----------------------------
    void applySafetyLogic() {

        bool going_forward  = last_raw_cmd_.linear.x > 0.0;
        bool going_backward = last_raw_cmd_.linear.x < 0.0;
        bool rotating       = last_raw_cmd_.angular.z != 0.0;

        bool obstacle_too_close = (min_distance_ < threshold_);

        // Attiva rollback se necessario
        if (!rollback_active_ && obstacle_too_close) {

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

        // Rollback in corso
        if (rollback_active_) {

            geometry_msgs::msg::Twist rb_cmd;

            if (last_raw_cmd_.linear.x > 0.0)
                rb_cmd.linear.x = -0.2;
            else if (last_raw_cmd_.linear.x < 0.0)
                rb_cmd.linear.x = 0.2;

            rb_cmd.angular.z = 0.0;

            cmd_pub_->publish(rb_cmd);

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

        // Comportamento normale
        cmd_pub_->publish(last_raw_cmd_);
    }

    // -----------------------------
    // PUBLISHER OBSTACLE INFO
    // -----------------------------
    void publishObstacleInfo() {
        assignment2::msg::ObstacleInfo msg;
        msg.min_distance = min_distance_;
        msg.direction = direction_;
        obstacle_pub_->publish(msg);
    }

    // -----------------------------
    // SERVICE: SET THRESHOLD
    // -----------------------------
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

    // -----------------------------
    // SERVICE: GET VEL AVG
    // -----------------------------
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
