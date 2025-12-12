#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class BebopControlNode : public rclcpp::Node {
public:
    BebopControlNode() : Node("bebop_control_node") {
        // bebop_autonomy parameters
        // MAKE SURE THESE MATCH
        this->declare_parameter("max_tilt_deg", 5.0);
        this->declare_parameter("max_vert_speed", 1.0);
        
        // PID gains
        this->declare_parameter("kp_xy", 0.1); 
        this->declare_parameter("ki_xy", 0.0); 
        this->declare_parameter("kd_xy", 0.00); 
        this->declare_parameter("kp_z", 0.1); 

        // Topics
        this->declare_parameter("odom_topic", "/bebop104/filtered_odom");
        this->declare_parameter("des_vel_topic", "/bebop104/cmd_vel_des");
        this->declare_parameter("cmd_vel_topic", "/bebop104/cmd_vel");
        this->declare_parameter("bebop_mode_topic", "/bebop104/mode");
        // this->declare_parameter<std::string>("odom_topic");
        // this->declare_parameter<std::string>("des_vel_topic");
        // this->declare_parameter<std::string>("cmd_vel_topic");
        
        // Load parameters
        max_tilt_rad_ = this->get_parameter("max_tilt_deg").as_double() * M_PI / 180.0;
        max_vert_speed_ = this->get_parameter("max_vert_speed").as_double();
        // Load gains
        kp_xy_ = this->get_parameter("kp_xy").as_double();
        ki_xy_ = this->get_parameter("ki_xy").as_double();
        kd_xy_ = this->get_parameter("kd_xy").as_double();
        kp_z_ = this->get_parameter("kp_z").as_double();

        // Subscribers
        // Odometry
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            std::bind(&BebopControlNode::odomCallback, this, std::placeholders::_1));
        // Desired velocity
        std::string des_vel_topic = this->get_parameter("des_vel_topic").as_string();
        des_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            des_vel_topic,10,
            std::bind(&BebopControlNode::desVelCallback, this, std::placeholders::_1));
        // Bebop mode (int)
        std::string bebop_mode_topic = this->get_parameter("bebop_mode_topic").as_string();
        bebop_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            bebop_mode_topic,10,
            std::bind(&BebopControlNode::bebopModeCallback, this, std::placeholders::_1));
        
        // Publisher
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        // Log
        RCLCPP_INFO(this->get_logger(), "Low-level Bebop controller active. Listening for %s", des_vel_topic.c_str());
    }

private:
    // State
    geometry_msgs::msg::Twist target_vel_world_; // Desired world frame vel
    Eigen::Vector3d current_vel_body_; // Current body frame vel
    Eigen::Quaterniond current_att_; // Orientation
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_cmd_time_;
    bool odom_received_ = false;
    bool cmd_received_ = false;

    // PID integral, derivative terms
    double err_sum_x_ = 0.0;
    double err_sum_y_ = 0.0;
    double last_vel_body_x_ = 0.0;
    double last_vel_body_y_ = 0.0;
    double last_err_y_ = 0.0;

    // Parameters
    double max_tilt_rad_;
    double max_vert_speed_;
    double kp_xy_, ki_xy_, kd_xy_;
    double kp_z_;
    int bebop_mode_ = 0;

    void desVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vel_world_ = *msg;
        last_cmd_time_ = this->now();
        cmd_received_ = true;
        controlLoop();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_time_ = this->now();
        odom_received_ = true;

        // Store body frame vel
        current_vel_body_ = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        // Store attitude
        current_att_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
    }

    void bebopModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        bebop_mode_ = msg->data;
    }   

    void controlLoop() {
        // Hover if no odom (either never received or stale)
        if (!odom_received_ || (this->now() - last_odom_time_).seconds() > 0.5) {
            stopDrone();
            RCLCPP_WARN(this->get_logger(), "Stale odometry.");
            return;
        }

        // Hover if no recent command
        if (!cmd_received_ || (this->now() - last_cmd_time_).seconds() > 1.0) {
            stopDrone();
            RCLCPP_WARN(this->get_logger(), "No recent command.");
            return;
        }

        // Hover if not in offboard mode (mode 1)
        if (bebop_mode_ != 1) {
            stopDrone();
            RCLCPP_WARN(this->get_logger(), "Not in offboard mode.");
            return;
        }

        // Coordinate transform
        // world -> body
        current_att_.normalize();

        // Convert target vel to body frame
        Eigen::Vector3d target_vel_body = current_att_.inverse() * Eigen::Vector3d(
            target_vel_world_.linear.x,
            target_vel_world_.linear.y,
            target_vel_world_.linear.z
        );

        // Calculate errors (body frame)
        double err_x = target_vel_body.x() - current_vel_body_.x();
        double err_y = target_vel_body.y() - current_vel_body_.y();
        double err_z = target_vel_body.z() - current_vel_body_.z(); 

        // Integral
        double dt = 0.02; // 50 Hz
        err_sum_x_ = std::clamp(err_sum_x_ + err_x * dt, -0.2,0.2);
        err_sum_y_ = std::clamp(err_sum_y_ + err_y * dt, -0.2,0.2);

        // Derivative
        double d_vel_x = (current_vel_body_.x() - last_vel_body_x_) / dt;
        double d_vel_y = (current_vel_body_.y() - last_vel_body_y_) / dt;
        last_vel_body_x_ = current_vel_body_.x();
        last_vel_body_y_ = current_vel_body_.y();

        // PID output (desired tilt in rad)
        double u_pitch = kp_xy_ * err_x + ki_xy_ * err_sum_x_ - kd_xy_ * d_vel_x;
        double u_roll  = kp_xy_ * err_y + ki_xy_ * err_sum_y_ - kd_xy_ * d_vel_y;
        double u_vert = kp_z_ * err_z;

        // Output
        geometry_msgs::msg::Twist cmd_vel;
        // Normalize
        cmd_vel.linear.x = std::clamp(u_pitch / max_tilt_rad_,-1.0,1.0);
        cmd_vel.linear.y = std::clamp(u_roll / max_tilt_rad_,-1.0,1.0);
        cmd_vel.linear.z = std::clamp(u_vert / max_vert_speed_,-1.0,1.0);
        cmd_vel.angular.z = std::clamp(target_vel_world_.angular.y, -1.0,1.0); // Yaw rate command directly
        
        // Publish
        cmd_vel_pub_->publish(cmd_vel);
    }

    void stopDrone() {
        // cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // Zero velocities
        // Reset integral terms
        err_sum_x_ = 0.0;
        err_sum_y_ = 0.0;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr des_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bebop_mode_sub_;
    // rclcpp::TimerBase::SharedPtr control_timer_;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopControlNode>());
  rclcpp::shutdown();
  return 0;
}