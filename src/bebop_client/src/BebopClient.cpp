#include "BebopClient.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

BebopClient::BebopClient() : Node("bebop_client_node"), flying_state_(landed), control_mode_(teleop) {
    RCLCPP_INFO(this->get_logger(), "Initializing Bebop Client Node");

    // Get my namespace (remove the slash with substr)
    bebop_id_ = std::string(this->get_namespace()).substr(1);

    // Initialize velocity command to zero
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.z = 0;

    state_pub_ = this->create_publisher<std_msgs::msg::Int32>("state", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("mode", 10);

    takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>("takeoff", 10);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("land", 10);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    tol_service_ = this->create_service<std_srvs::srv::SetBool>("tol", std::bind(&BebopClient::tol_callback, this, _1, _2));
    mode_service_ = this->create_service<std_srvs::srv::SetBool>("set_mode", std::bind(&BebopClient::mode_callback, this, _1, _2));

    // Publish bebop state (flying state and mode) at 1 hz
    state_timer_ = this->create_wall_timer(1s, std::bind(&BebopClient::publish_state, this));

    // Publish cmd_vel setpoints at 50Hz
    cmd_vel_timer_ = this->create_wall_timer(20ms, std::bind(&BebopClient::publish_cmd_vel, this));

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/bebop_server_node/cmd_teleop", 10, std::bind(&BebopClient::teleop_callback, this, _1));
}

void BebopClient::publish_state() {
    std_msgs::msg::Int32 state_msg, mode_msg;

    state_msg.data = flying_state_;
    state_pub_->publish(state_msg);

    mode_msg.data = control_mode_;
    mode_pub_->publish(mode_msg);
}

// "Velocity" (really attitude) timer callback
void BebopClient::publish_cmd_vel() {
    vel_pub_->publish(cmd_vel_);
}

void BebopClient::teleop_callback(const geometry_msgs::msg::Twist::SharedPtr teleop_msg) {
    // If in teleop mode, feed the attitude command straight through
    if (control_mode_ == teleop) {
        // todo:: velocity controller and safety filtering
        cmd_vel_ = *teleop_msg;
    }
}

// Takeoff and landing (TOL) service request
void BebopClient::tol_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    // If request is true, takeoff, otherwise land
    std::string tol_str = request->data ? "Takeoff" : "Landing";
    RCLCPP_INFO(this->get_logger(), "%s requested", tol_str.c_str());

    // Respond with success message
    response->success = true;
    response->message = bebop_id_ + std::string(": ") + tol_str + std::string(" request succeeded.");

    if (request->data) {
        request_takeoff();
    } else {
        request_landing();
    }
}

// Autonomous (vs teleop) service request
void BebopClient::mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    // If request is true, takeoff, otherwise land
    std::string ctrl_mode_str = request->data ? "teleop" : "autonomous";

    RCLCPP_INFO(this->get_logger(), "Switch to %s mode requested", ctrl_mode_str.c_str());

    if (request->data) {
        control_mode_ = autonomous;
    } else {
        control_mode_ = teleop;
    }
    publish_state();

    // Respond with success message
    response->success = true;
    response->message = bebop_id_ + std::string(": ") + std::string(" control mode switched to ").append(ctrl_mode_str);
}

void BebopClient::request_takeoff() {
    if (flying_state_ == landed) {
        RCLCPP_WARN(this->get_logger(), "Sending takeoff request to bebop.");

        flying_state_ = taking_off;
        publish_state();

        std_msgs::msg::Empty takeoff_msg;
        takeoff_pub_->publish(takeoff_msg);

        // After 4 seconds, assume takeoff succeded (no takeoff confirmation, unfortunately)
        // one_shot_timer_1_ = this->create_wall_timer(
        //     std::chrono::seconds(1),
        //     [this]() {
        //         if (timer_1_ready_) {
        //             flying_state_ = flying;
        //             publish_state();

        //             one_shot_timer_1_->cancel();
        //             timer_1_ready_ = false;
        //         } else {
        //             timer_1_ready_ = true;
        //         }
        //     }
        // );

        rclcpp::sleep_for(2s);
        flying_state_ = flying;
        publish_state();

    } else { 
        RCLCPP_WARN(this->get_logger(), "Takeoff request ignored - bebop already flying.");
    }
}

void BebopClient::request_landing() {
    if (flying_state_ == flying) {
        RCLCPP_WARN(this->get_logger(), "Sending landing request to bebop.");

        flying_state_ = landing;
        publish_state();

        // Send zero velocity command
        geometry_msgs::msg::Twist cmd_vel_zero;
        vel_pub_->publish(cmd_vel_zero);

        // After 1 second of zero velocity, publish the land message
        rclcpp::sleep_for(1s);
        std_msgs::msg::Empty land_msg;
        land_pub_->publish(land_msg);

        // After 2 seconds, assume landing succeeded
        rclcpp::sleep_for(2s);
        flying_state_ = landed;
        publish_state();

    } else {
        RCLCPP_WARN(this->get_logger(), "Landing request ignored - bebop is not flying.");
    }
}