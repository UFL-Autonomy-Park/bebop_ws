#ifndef BEBOP_CLIENT_HPP
#define BEBOP_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>

// Message types
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>

//Service types
#include <std_srvs/srv/set_bool.hpp>
// #include <std_srvs/srv/trigger.hpp>

class BebopClient : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_, land_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_, mode_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tol_service_, mode_service_;

    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

    enum FlyingState {
        landed = 0,
        taking_off,
        flying,
        landing
    };
    FlyingState flying_state_;

    // Note: Could use a bool here but I'm leaving the option to implement additional control modes
    enum ControlMode {
        teleop = 0,
        autonomous
    };
    ControlMode control_mode_;

    std::string bebop_id_;
    geometry_msgs::msg::Twist cmd_vel_;

    void publish_state();
    void publish_cmd_vel();

    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr joy_msg);
    void tol_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void request_takeoff();
    void request_landing();

public:
    BebopClient();
};

#endif
