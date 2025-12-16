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
        // --- Parameters ---
        auto parent_frame = this->declare_parameter<std::string>("frames.parent");
        auto child_frame = this->declare_parameter<std::string>("frames.child");
        auto input_topic = this->declare_parameter<std::string>("topics.input_pose");
        double freq = this->declare_parameter<double>("frequency");
        double dt = 1.0 / freq;

        // Filter Params
        double lev_C = this->declare_parameter<double>("levant.C");
        double rho_k1 = this->declare_parameter<double>("rho.k1");
        double rho_k2 = this->declare_parameter<double>("rho.k2");
        double rho_k3 = this->declare_parameter<double>("rho.k3");
        double rho_alpha = this->declare_parameter<double>("rho.alpha");
        double rushi_rho_alpha = this->declare_parameter<double>("rushi_rho.alpha");
        double rushi_rho_beta = this->declare_parameter<double>("rushi_rho.beta");
        double rushi_rho_k = this->declare_parameter<double>("rushi_rho.k");
        double dirty_N = this->declare_parameter<double>("baseline.dirty_N");

        // --- Initialization ---
        levant_ = std::make_unique<mocap_filters::LevantFilter>(lev_C, dt);
        rho_ = std::make_unique<mocap_filters::RhoFilter>(dt, 3, rho_alpha, rho_k1, rho_k2, rho_k3);
        rushi_rho_ = std::make_unique<mocap_filters::rushi_RhoFilter>(dt, 3, rushi_rho_alpha, rushi_rho_beta, rushi_rho_k);
        
        // New Classes
        numeric_ = std::make_unique<mocap_filters::NumericalDifferentiation>(dt);
        dirty_ = std::make_unique<mocap_filters::DirtyDerivative>(dirty_N, dt);

        // --- Communication ---
        pub_levant_    = this->create_publisher<nav_msgs::msg::Odometry>("odom/levant", 10);
        pub_rho_       = this->create_publisher<nav_msgs::msg::Odometry>("odom/rho", 10);
        pub_dirty_     = this->create_publisher<nav_msgs::msg::Odometry>("odom/dirty", 10);
        pub_numeric_   = this->create_publisher<nav_msgs::msg::Odometry>("odom/numeric", 10);
        pub_rushi_rho_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/rushi_rho", 10);

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
    void timer_callback() {
        if (!has_data_) return;

        Eigen::Vector3d p_raw_mocap_frame;
        Eigen::Quaterniond q_raw_mocap_frame;
        tf2::fromMsg(latest_msg_->pose.position, p_raw_mocap_frame);
        tf2::fromMsg(latest_msg_->pose.orientation, q_raw_mocap_frame);

        // --- Propagate All Filters ---
        levant_->propagate(p_raw_mocap_frame);
        rho_->propagate_filter(p_raw_mocap_frame);
        rushi_rho_->propagate_filter(p_raw_mocap_frame);
        numeric_->propagate(p_raw_mocap_frame);
        dirty_->propagate(p_raw_mocap_frame);

        // --- Publish Results ---
        Eigen::Vector3d w_zero = Eigen::Vector3d::Zero();

        publish_odometry_message(pub_levant_, levant_->get_position_estimate(), levant_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_rho_, rho_->get_position_estimate(), rho_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_rushi_rho_, rushi_rho_->get_position_estimate(), rushi_rho_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);

        // For baselines, position estimate IS the raw measurement
        publish_odometry_message(pub_dirty_, p_raw_mocap_frame, dirty_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
        publish_odometry_message(pub_numeric_, p_raw_mocap_frame, numeric_->get_velocity_estimate(), w_zero, q_raw_mocap_frame);
    }

    void publish_odometry_message(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub, 
                                  const Eigen::Vector3d& p_est_mocap, 
                                  const Eigen::Vector3d& v_est_mocap,
                                  const Eigen::Vector3d& w_est_mocap,
                                  const Eigen::Quaterniond& q_att_mocap) 
    {
        nav_msgs::msg::Odometry odom = odom_msg_template_;
        odom.header.stamp = this->now(); 

        odom.pose.pose.position = tf2::toMsg(p_est_mocap);
        odom.pose.pose.orientation = tf2::toMsg(q_att_mocap); 

        // Rotate Mocap Velocity -> Body Frame
        Eigen::Vector3d v_est_body = q_att_mocap.inverse() * v_est_mocap;
        
        odom.twist.twist.linear.x = v_est_body.x();
        odom.twist.twist.linear.y = v_est_body.y();
        odom.twist.twist.linear.z = v_est_body.z();

        if (w_est_mocap.norm() > 1e-6) {
            Eigen::Vector3d w_est_body = q_att_mocap.inverse() * w_est_mocap;
            odom.twist.twist.angular.x = w_est_body.x();
            odom.twist.twist.angular.y = w_est_body.y();
            odom.twist.twist.angular.z = w_est_body.z();
        } else {
            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;
        }

        pub->publish(odom);
    }

    bool has_data_;
    nav_msgs::msg::Odometry odom_msg_template_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_msg_;

    // Filter Objects
    std::unique_ptr<mocap_filters::LevantFilter> levant_;
    std::unique_ptr<mocap_filters::RhoFilter> rho_;
    std::unique_ptr<mocap_filters::rushi_RhoFilter> rushi_rho_;
    std::unique_ptr<mocap_filters::NumericalDifferentiation> numeric_;
    std::unique_ptr<mocap_filters::DirtyDerivative> dirty_;

    // ROS Objects
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_levant_, pub_rho_, pub_rushi_rho_, pub_dirty_, pub_numeric_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<MocapFiltersNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}