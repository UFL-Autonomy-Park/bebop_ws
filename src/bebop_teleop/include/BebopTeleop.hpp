#ifndef BEBOP_TELEOP_HPP
#define BEBOP_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>

class BebopTeleop : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_, land_pub_;

	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::TimerBase::SharedPtr setpoint_timer_;

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
        Button takeoff;
        Button land;
    } buttons_;

    //Button state for debouncing
    struct ButtonState {
        ButtonState() : state(0) {}
        int state;
    };

    struct {
        ButtonState takeoff;
        ButtonState land;
    } button_state_;

    enum bebop_flying_state {
        landed = 0,
        taking_off,
        hovering,
        flying,
        landing,
        emergency,
        user_takeoff,
        motor_ramping,
        emergency_landing
    };

    geometry_msgs::msg::Twist setpoint_vel_;

    double get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis);
    int get_joy_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

    // void flying_state_callback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChangedConstPtr &flying_state_msg);

    void publish_setpoint();
    void request_takeoff();
    void request_landing();
    
public:
	BebopTeleop();
};

#endif