#include "BebopTeleop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

BebopTeleop::BebopTeleop() : Node("bebop_teleop_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Bebop Teleop Node");

    // Joy axis and button config
    this->declare_parameter("x_axis", -1);
    this->declare_parameter("y_axis", -1);
    this->declare_parameter("z_axis", -1);
    this->declare_parameter("yaw_axis", -1);
    this->declare_parameter("takeoff_button", -1);
    this->declare_parameter("land_button", -1);

    this->declare_parameter("x_vel_max", 1.0);
    this->declare_parameter("y_vel_max", 1.0);
    this->declare_parameter("z_vel_max", 1.0);
    this->declare_parameter("yaw_vel_max", 1.0);

    this->get_parameter("x_axis", axes_.x.axis);
    this->get_parameter("y_axis", axes_.y.axis);
    this->get_parameter("z_axis", axes_.z.axis);
    this->get_parameter("yaw_axis", axes_.yaw.axis);

    this->get_parameter("takeoff_button", buttons_.takeoff.button);
    this->get_parameter("land_button", buttons_.land.button);

    this->get_parameter("x_vel_max", axes_.x.factor);
    this->get_parameter("y_vel_max", axes_.y.factor);
    this->get_parameter("z_vel_max", axes_.z.factor);
    this->get_parameter("yaw_vel_max", axes_.yaw.factor);

    RCLCPP_INFO(this->get_logger(), "Loaded controller parameters:\nX: %d, Y: %d, Z: %d, Yaw: %d, Takeoff: %d, Land: %d", 
        axes_.x.axis, axes_.y.axis, axes_.z.axis, axes_.yaw.axis, buttons_.takeoff.button, buttons_.land.button);

    // Publish setpoints at 50Hz
    setpoint_timer_ = this->create_wall_timer(20ms, std::bind(&BebopTeleop::publish_setpoint, this));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("land", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&BebopTeleop::joy_callback, this, _1));
}

double BebopTeleop::get_joy_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Axis &axis) {
    if (axis.axis < 0 || std::abs(axis.axis) > (int)joy_msg->axes.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Axis %d out of range, joy has %d axes", axis.axis, (int)joy_msg->axes.size());
        return -1;
    }

    double output = joy_msg->axes[std::abs(axis.axis)] * axis.factor + axis.offset;

    return output;
}

int BebopTeleop::get_joy_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
    if (button.button < 0 || button.button > (int)joy_msg->buttons.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.button, (int)joy_msg->buttons.size());
        return -1;
    }

    return joy_msg->buttons[button.button];
}

void BebopTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

    // Check for button presses
    int takeoff_button_state = get_joy_button(joy_msg, buttons_.takeoff);
    if (takeoff_button_state != button_state_.takeoff.state) {
        if (takeoff_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Takeoff button pressed");
            request_takeoff();
        }

        button_state_.takeoff.state = takeoff_button_state;
    }

    int land_button_state = get_joy_button(joy_msg, buttons_.land);
    if (land_button_state != button_state_.land.state) {
        if (land_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Land button pressed");
            request_landing();
        }

        button_state_.land.state = land_button_state;
    }

    // Read stick values
    setpoint_vel_.linear.x = get_joy_axis(joy_msg, axes_.x);
    setpoint_vel_.linear.y = get_joy_axis(joy_msg, axes_.y);
    setpoint_vel_.linear.z = get_joy_axis(joy_msg, axes_.z);
    setpoint_vel_.angular.z = get_joy_axis(joy_msg, axes_.yaw);
}

void BebopTeleop::publish_setpoint() {
    rclcpp::Time now = this->get_clock()->now();
    vel_pub_->publish(setpoint_vel_);
}



void BebopTeleop::request_takeoff() {
    // if (!is_flying()) {
    RCLCPP_WARN(this->get_logger(), "Takeoff requested");

    std_msgs::msg::Empty takeoff_msg;
    takeoff_pub_->publish(takeoff_msg);
    // }
}

void BebopTeleop::request_landing() {
    RCLCPP_WARN(this->get_logger(), "Landing requested");

    // Send zero velocity command, delay 1 second, then land the drone
    geometry_msgs::msg::Twist cmd_vel_zero;
    vel_pub_->publish(cmd_vel_zero);
    rclcpp::sleep_for(1s);
    std_msgs::msg::Empty land_msg;
    land_pub_->publish(land_msg);
}

// void BebopTeleop::flying_state_callback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChangedConstPtr &state_msg) {
//     //Update the flying state
//     flying_state_ = static_cast<bebop_flying_state>(state_msg->state);

//     switch(flying_state_) {
//     case landed:
//         ROS_WARN("%s BebopController: Switched to landed state!", bebop_prefix_.c_str());
//         break;
//     case taking_off:
//         ROS_WARN("%s BebopController: Switched to takeoff state!", bebop_prefix_.c_str());
//         break;
//     case hovering:
//         ROS_WARN("%s BebopController: Switched to hover state!", bebop_prefix_.c_str());
//         break;
//     case flying:
//         ROS_WARN("%s BebopController: Switched to flying state!", bebop_prefix_.c_str());
//         break;
//     case landing:
//         ROS_WARN("%s BebopController: Switched to landing state!", bebop_prefix_.c_str());
//         break;
//     case emergency:
//         ROS_ERROR("%s BebopController: Switched to emergency state!", bebop_prefix_.c_str());
//         break;
//     case user_takeoff:
//         ROS_WARN("%s BebopController: Switched to user takeoff state!", bebop_prefix_.c_str());
//         break;
//     case motor_ramping:
//         ROS_WARN("%s BebopController: Switched to motor ramping state!", bebop_prefix_.c_str());
//         break;
//     case emergency_landing:
//         ROS_ERROR("%s BebopController: Switched to emergency landing state!", bebop_prefix_.c_str());
//         break;
//     default:
//         ROS_WARN("%s BebopController: Invalid state received.", bebop_prefix_.c_str());
//     }
// }
