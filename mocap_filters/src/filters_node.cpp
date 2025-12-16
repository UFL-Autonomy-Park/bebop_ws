#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "mocap_filters/LevantFilter.hpp"
#include "mocap_filters/RhoFilter.hpp"
#include "mocap_filters/rushi_RhoFilter.hpp"

using namespace std::chrono_literals;

class MocapFiltersNode : public rclcpp::Node {
public:
    MocapFiltersNode() : Node("mocap_filters_node"), has_data_(false) {
        // --- Parameters ---
        auto parent_frame = this->declare_parameter<std::string>("frames.parent");
        auto child_frame = this->declare_parameter<std::string>("frames.child");
        auto input_topic = this->declare_parameter<std::string>("topics.input_pose");
        double freq = this->declare_parameter<double>("frequency"); // Hz
        dt_ = 1.0 / freq; // seconds

        // Levant Params
        double lev_C = this->declare_parameter<double>("levant.C");
        
        // Rho Params
        double rho_k1 = this->declare_parameter<double>("rho.k1");
        double rho_k2 = this->declare_parameter<double>("rho.k2");
        double rho_k3 = this->declare_parameter<double>("rho.k3");
        double rho_alpha = this->declare_parameter<double>("rho.alpha");

        // Rushi's Rho Params
        double rushi_rho_alpha = this->declare_parameter<double>("rushi_rho.alpha");
        double rushi_rho_beta = this->declare_parameter<double>("rushi_rho.beta");
        double rushi_rho_k = this->declare_parameter<double>("rushi_rho.k");

        // Baseline Params
        dirty_N_ = this->declare_parameter<double>("baseline.dirty_N");

        // --- Initialization ---
        levant_ = std::make_unique<mocap_filters::LevantFilter>(lev_C, dt_);
        rho_ = std::make_unique<mocap_filters::RhoFilter>(dt_, 3, rho_alpha, rho_k1, rho_k2, rho_k3);
        rushi_rho_ = std::make_unique<mocap_filters::rushi_RhoFilter>(dt_, 3, rushi_rho_alpha, rushi_rho_beta, rushi_rho_k);

        v_dirty_mocap_frame_.setZero();
        p_prev_mocap_frame_numeric_.setZero();
        p_prev_mocap_frame_dirty_.setZero();

        pub_levant_    = this->create_publisher<nav_msgs::msg::Odometry>("odom/levant", 10);
        pub_rho_       = this->create_publisher<nav_msgs::msg::Odometry>("odom/rho", 10);
        pub_dirty_     = this->create_publisher<nav_msgs::msg::Odometry>("odom/dirty", 10);
        pub_numeric_   = this->create_publisher<nav_msgs::msg::Odometry>("odom/numeric", 10);
        pub_rushi_rho_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/rushi_rho", 10);

        // Subscriber: Directly updates member variable
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                latest_msg_ = msg;
                has_data_ = true;
            });

        // Timer: Runs filters at fixed freq
        auto period = std::chrono::duration<double>(dt_);
        timer_ = this->create_wall_timer(period, std::bind(&MocapFiltersNode::timer_callback, this));

        odom_msg_template_.header.frame_id = parent_frame;
        odom_msg_template_.child_frame_id = child_frame;
    }

private:
    void timer_callback() {
        if (!has_data_) return;

        // 1. Raw Mocap Frame (Y-up)
        Eigen::Vector3d p_raw_mocap_frame;
        Eigen::Quaterniond q_raw_mocap_frame;
        tf2::fromMsg(latest_msg_->pose.position, p_raw_mocap_frame);
        tf2::fromMsg(latest_msg_->pose.orientation, q_raw_mocap_frame);

        // 2. Initialization on first valid data
        if (!initialized_) {
            p_prev_mocap_frame_numeric_ = p_raw_mocap_frame;
            p_prev_mocap_frame_dirty_ = p_raw_mocap_frame;
            initialized_ = true;
        }

        // --- Filters Propagate/Update (Calculations in Mocap Frame) ---
        levant_->propagate(p_raw_mocap_frame);
        rho_->propagate_filter(p_raw_mocap_frame);
        rushi_rho_->propagate_filter(p_raw_mocap_frame);

        // --- Baselines (Calculations in Mocap Frame) ---
        // 1. Numerical Differentiation (Backward Euler)
        Eigen::Vector3d v_numeric_mocap_frame = (p_raw_mocap_frame - p_prev_mocap_frame_numeric_) / dt_;
        p_prev_mocap_frame_numeric_ = p_raw_mocap_frame;

        // 2. Dirty Derivative (Tustin)
        double tau = 1.0 / dirty_N_;
        double a1 = (2.0 * tau - dt_) / (2.0 * tau + dt_);
        double a2 = 2.0 / (2.0 * tau + dt_);
        
        v_dirty_mocap_frame_ = a1 * v_dirty_mocap_frame_ + a2 * (p_raw_mocap_frame - p_prev_mocap_frame_dirty_);
        p_prev_mocap_frame_dirty_ = p_raw_mocap_frame;

        // --- Transforms ---
        // Rotation: Mocap (Y-up) -> ENU (Z-up)
        // This is a +90 deg rotation about X
        static const Eigen::Quaterniond q_mocap_to_enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        
        // Full orientation in ENU frame
        Eigen::Quaterniond q_est_enu_frame = q_mocap_to_enu * q_raw_mocap_frame;
        q_est_enu_frame.normalize();

        // Helper to Transform & Publish
        auto publish_odom = [&](rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub, 
                                const Eigen::Vector3d& p_est_mocap_frame, 
                                const Eigen::Vector3d& v_est_mocap_frame,
                                const Eigen::Vector3d& w_est_mocap_frame = Eigen::Vector3d::Zero()) {
            
            nav_msgs::msg::Odometry odom = odom_msg_template_;
            odom.header.stamp = this->now(); 

            // Position: Rotate Mocap Frame -> ENU Frame
            Eigen::Vector3d p_est_enu_frame = q_mocap_to_enu * p_est_mocap_frame;
            odom.pose.pose.position = tf2::toMsg(p_est_enu_frame);
            
            // Orientation: We usually pass the raw Mocap orientation or the ENU orientation
            // Passing the transformed ENU orientation here:
            odom.pose.pose.orientation = tf2::toMsg(q_est_enu_frame); 

            // Velocity: 
            // 1. Rotate Mocap Frame -> ENU Frame
            Eigen::Vector3d v_est_enu_frame = q_mocap_to_enu * v_est_mocap_frame;
            
            // 2. Rotate ENU Frame -> Body Frame (Inverse of Body Orientation)
            // This aligns velocity with the drone's body axes (x-forward, y-left, z-up)
            Eigen::Vector3d v_est_body_frame = q_est_enu_frame.inverse() * v_est_enu_frame;
            
            odom.twist.twist.linear.x = v_est_body_frame.x();
            odom.twist.twist.linear.y = v_est_body_frame.y();
            odom.twist.twist.linear.z = v_est_body_frame.z();

            // Angular Velocity (if available)
            if (w_est_mocap_frame.norm() > 1e-6) {
                Eigen::Vector3d w_est_enu_frame = q_mocap_to_enu * w_est_mocap_frame;
                Eigen::Vector3d w_est_body_frame = q_est_enu_frame.inverse() * w_est_enu_frame;
                odom.twist.twist.angular.x = w_est_body_frame.x();
                odom.twist.twist.angular.y = w_est_body_frame.y();
                odom.twist.twist.angular.z = w_est_body_frame.z();
            }

            pub->publish(odom);
        };

        // Publish
        publish_odom(pub_levant_, levant_->get_position_estimate(), levant_->get_velocity_estimate());
        publish_odom(pub_rho_, rho_->get_position_estimate(), rho_->get_velocity_estimate());
        publish_odom(pub_rushi_rho_, rushi_rho_->get_position_estimate(), rushi_rho_->get_velocity_estimate());

        publish_odom(pub_dirty_, p_raw_mocap_frame, v_dirty_mocap_frame_);
        publish_odom(pub_numeric_, p_raw_mocap_frame, v_numeric_mocap_frame);
    }

    bool initialized_ = false;
    bool has_data_ = false;
    double dt_;
    Eigen::Vector3d v_dirty_mocap_frame_, p_prev_mocap_frame_dirty_, p_prev_mocap_frame_numeric_;
    double dirty_N_;
    nav_msgs::msg::Odometry odom_msg_template_;
    
    geometry_msgs::msg::PoseStamped::SharedPtr latest_msg_;

    std::unique_ptr<mocap_filters::LevantFilter> levant_;
    std::unique_ptr<mocap_filters::RhoFilter> rho_;
    std::unique_ptr<mocap_filters::rushi_RhoFilter> rushi_rho_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_levant_, pub_rho_, pub_rushi_rho_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_dirty_, pub_numeric_;
    
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