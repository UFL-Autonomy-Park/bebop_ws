#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "mocap_filters/LevantFilter.hpp"
#include "mocap_filters/RhoFilter.hpp"
#include "mocap_filters/rushi_RhoFilter.hpp"
#include "mocap_filters/NumericalDifferentiation.hpp"
#include "mocap_filters/DirtyDerivative.hpp"

using namespace std::chrono_literals;

class MocapFiltersNode : public rclcpp::Node {
public:
    MocapFiltersNode() : Node("mocap_filters_node"), has_data_(false) {
        auto parent_frame = this->declare_parameter<std::string>("frames.parent");
        auto child_frame = this->declare_parameter<std::string>("frames.child");
        auto input_topic = this->declare_parameter<std::string>("topics.input_pose");
        double freq = this->declare_parameter<double>("frequency");
        if (freq <= 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Invalid frequency parameter: %f. Frequency must be strictly positive.",freq);
            throw std::invalid_argument("Frequency must be greater than zero");
        }
        double dt = 1.0 / freq;

        double lev_C = this->declare_parameter<double>("levant.C");
        double rho_k1 = this->declare_parameter<double>("rho.k1");
        double rho_k2 = this->declare_parameter<double>("rho.k2");
        double rho_k3 = this->declare_parameter<double>("rho.k3");
        double rho_alpha = this->declare_parameter<double>("rho.alpha");
        double rushi_rho_alpha = this->declare_parameter<double>("rushi_rho.alpha");
        double rushi_rho_beta = this->declare_parameter<double>("rushi_rho.beta");
        double rushi_rho_k = this->declare_parameter<double>("rushi_rho.k");
        double dirty_N = this->declare_parameter<double>("baseline.dirty_N");

        levant_ = std::make_unique<mocap_filters::LevantFilter>(lev_C, dt);
        rho_ = std::make_unique<mocap_filters::RhoFilter>(dt, 3, rho_alpha, rho_k1, rho_k2, rho_k3);
        rushi_rho_ = std::make_unique<mocap_filters::rushi_RhoFilter>(dt, 3, rushi_rho_alpha, rushi_rho_beta, rushi_rho_k);
        numeric_ = std::make_unique<mocap_filters::NumericalDifferentiation>(dt);
        dirty_ = std::make_unique<mocap_filters::DirtyDerivative>(dirty_N, dt);

        pub_levant_    = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom/levant", 10);
        pub_rho_       = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom/rho", 10);
        pub_dirty_     = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom/dirty", 10);
        pub_numeric_   = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom/numeric", 10);
        pub_rushi_rho_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom/rushi_rho", 10);
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                latest_msg_ = msg;
                has_data_ = true;
            });

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), 
                                         std::bind(&MocapFiltersNode::timer_callback, this));

        odom_msg_template_.header.frame_id = parent_frame;
        odom_msg_template_.child_frame_id = child_frame;
    }

private:
    bool has_data_;
    nav_msgs::msg::Odometry odom_msg_template_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_msg_;

    std::unique_ptr<mocap_filters::LevantFilter> levant_;
    std::unique_ptr<mocap_filters::RhoFilter> rho_;
    std::unique_ptr<mocap_filters::rushi_RhoFilter> rushi_rho_;
    std::unique_ptr<mocap_filters::NumericalDifferentiation> numeric_;
    std::unique_ptr<mocap_filters::DirtyDerivative> dirty_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_levant_, pub_rho_, pub_rushi_rho_, pub_dirty_, pub_numeric_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
        if (!has_data_) return;

        Eigen::Vector3d p_raw_mocap_frame;
        Eigen::Quaterniond q_raw_mocap_frame;
        tf2::fromMsg(latest_msg_->pose.position, p_raw_mocap_frame);
        tf2::fromMsg(latest_msg_->pose.orientation, q_raw_mocap_frame);

        levant_->propagate_filter(p_raw_mocap_frame);
        rho_->propagate_filter(p_raw_mocap_frame);
        rushi_rho_->propagate_filter(p_raw_mocap_frame);
        numeric_->propagate(p_raw_mocap_frame);
        dirty_->propagate(p_raw_mocap_frame);

        // These filters do not estimate angular velocity; set to zero
        // For numerical differentiation and dirty derivative, position estimate is just the raw measurement
        // All orientations are raw mocap frame orientations (no filtering)
        Eigen::Vector3d w_zero = Eigen::Vector3d::Zero();
        publish_odometry_message(pub_levant_, levant_->get_position_estimate(), levant_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_rho_, rho_->get_position_estimate(), rho_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_rushi_rho_, rushi_rho_->get_position_estimate(), rushi_rho_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_dirty_, p_raw_mocap_frame, dirty_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_numeric_, p_raw_mocap_frame, numeric_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
    }

    void publish_odometry_message(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub, 
                                  const Eigen::Vector3d& p_estimated_mocap_frame, 
                                  const Eigen::Vector3d& v_estimated_mocap_frame,
                                  const Eigen::Vector3d& w_estimated_mocap_frame,
                                  const Eigen::Quaterniond& q_raw_mocap_frame) 
    {
        nav_msgs::msg::Odometry odom = odom_msg_template_;
        odom.header.stamp = this->now();

        odom.pose.pose.position = tf2::toMsg(p_estimated_mocap_frame);
        odom.pose.pose.orientation = tf2::toMsg(q_raw_mocap_frame); 

        // Transform mocap frame velocities to body frame
        Eigen::Vector3d v_estimated_body = q_raw_mocap_frame.inverse() * v_estimated_mocap_frame;
        
        odom.twist.twist.linear.x = v_estimated_body.x();
        odom.twist.twist.linear.y = v_estimated_body.y();
        odom.twist.twist.linear.z = v_estimated_body.z();

        odom.twist.twist.angular.x = w_estimated_mocap_frame.x();
        odom.twist.twist.angular.y = w_estimated_mocap_frame.y();
        odom.twist.twist.angular.z = w_estimated_mocap_frame.z();

        pub->publish(odom);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    
    try {
        auto node = std::make_shared<MocapFiltersNode>();
        exec.add_node(node);
        exec.spin();
    } catch (const std::exception &e) {
        // This will print "Parameter 'frames.parent' not set" instead of just crashing!
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}