#ifndef BEBOP_SERVER_HPP
#define BEBOP_SERVER_HPP

#include <regex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>

//Service types
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

class BebopServer : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> state_subs_;
    std::map<int, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> mode_subs_;

    std::map<int, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> tol_clients_;
    std::map<int, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> mode_clients_;

    rclcpp::TimerBase::SharedPtr scan_timer_, teleop_timer_;

    // Controller axis and button structs
    struct Axis {
        Axis() : axis(0), factor(0.0), offset(0.0) {}

        int axis;
        double factor;
        double offset;
    };

    struct {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    struct Button {
        Button() : button(0) {}
        int button;
    };

    struct {
        Button mode;
        Button takeoff;
        Button land;
    } buttons_;

    //Button state for debouncing
    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        ButtonState mode;
        ButtonState takeoff;
        ButtonState land;
    } button_state_;

    double selector_state_;
    int teleop_id_;

    enum FlyingState {
        landed = 0,
        taking_off,
        flying,
        landing
    };

    enum ControlMode {
        teleop = 0,
        autonomous
    };

    //Bebop storage struct
    struct Bebop {
        FlyingState flying_state;
        ControlMode control_mode;
    };

    // Map of connected bebops
    std::map<int, Bebop> bebops_;

    // Teleop attitude setpoint sent to the bebop client
    geometry_msgs::msg::Twist teleop_cmd_;

    void scan_bebops();

    double get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);
    int get_joy_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void publish_teleop();
    void takeoff_all_bebops();
    void land_all_bebops();

    void tol_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
    void mode_response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);

public:
    BebopServer();
};

#endif