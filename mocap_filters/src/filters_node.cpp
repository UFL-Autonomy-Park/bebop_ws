#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "mocap_filters/LevantFilter.hpp"
#include "mocap_filters/KalmanFilter.hpp"
#include "mocap_filters/RhoFilter.hpp"

using namespace std::chrono_literals;

class MocapFiltersNode : public rclcpp::Node {
public:
    MocapFiltersNode() : Node("mocap_filters_node"), has_data_(false) {
        // --- Parameters ---
        auto parent_frame = this->declare_parameter<std::string>("frames.parent");
        auto child_frame = this->declare_parameter<std::string>("frames.child");
        auto input_topic = this->declare_parameter<std::string>("topics.input_pose");
        double freq = this->declare_parameter<double>("frequency"); // Hz
        dt_ = 1.0 / freq;

        // Levant Params
        double lev_C = this->declare_parameter<double>("levant.C");
        
        // Rho Params
        double rho_k1 = this->declare_parameter<double>("rho.k1");
        double rho_k2 = this->declare_parameter<double>("rho.k2");
        double rho_k3 = this->declare_parameter<double>("rho.k3");
        double rho_alpha = this->declare_parameter<double>("rho.alpha");

        // Kalman Params
        double kal_q_pos = this->declare_parameter<double>("kalman.Q_pos");
        double kal_q_vel = this->declare_parameter<double>("kalman.Q_vel");

        // Baseline Params
        dirty_N_ = this->declare_parameter<double>("baseline.dirty_N");

        // --- Initialization ---
        levant_ = std::make_unique<mocap_filters::LevantFilter>(lev_C, dt_);
        rho_ = std::make_unique<mocap_filters::RhoFilter>(dt_, 3, rho_alpha, rho_k1, rho_k2, rho_k3);
        kalman_ = std::make_unique<mocap_filters::KalmanFilter>();

        Eigen::Matrix<double, 12, 12> u_cov = Eigen::Matrix<double, 12, 12>::Zero();
        u_cov.topLeftCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * kal_q_pos;
        u_cov.bottomRightCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * kal_q_vel;
        kalman_->init(u_cov, Eigen::Matrix<double, 6, 6>::Identity() * 1e-5, freq);

        dirty_vel_.setZero();
        last_pos_numeric_.setZero();
        last_pos_dirty_.setZero();

        // --- Communication ---
        pub_levant_  = this->create_publisher<nav_msgs::msg::Odometry>("odom/levant", 10);
        pub_rho_     = this->create_publisher<nav_msgs::msg::Odometry>("odom/rho", 10);
        pub_kalman_  = this->create_publisher<nav_msgs::msg::Odometry>("odom/kalman", 10);
        pub_dirty_   = this->create_publisher<nav_msgs::msg::Odometry>("odom/dirty", 10);
        pub_numeric_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/numeric", 10);

        // Subscriber: Only updates the latest data buffer
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_msg_ = msg;
                has_data_ = true;
            });

        // Timer: Runs the filters at strict fixed frequency
        auto period = std::chrono::duration<double>(dt_);
        timer_ = this->create_wall_timer(period, std::bind(&MocapFiltersNode::timer_callback, this));

        odom_msg_template_.header.frame_id = parent_frame;
        odom_msg_template_.child_frame_id = child_frame;
    }

private:
    void timer_callback() {
        if (!has_data_) return;

        // 1. Thread-safe data retrieval
        geometry_msgs::msg::PoseStamped::SharedPtr msg_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            msg_copy = latest_msg_; 
        } 

        double t = this->now().seconds(); 
        
        Eigen::Vector3d p_raw;
        Eigen::Quaterniond q_raw;
        tf2::fromMsg(msg_copy->pose.position, p_raw);
        tf2::fromMsg(msg_copy->pose.orientation, q_raw);

        // 2. Initialization on first valid data
        if (!initialized_) {
            last_pos_numeric_ = p_raw;
            last_pos_dirty_ = p_raw;
            initialized_ = true;
        }

        // --- Filters Propagate/Update ---
        levant_->propagate(p_raw);
        
        Eigen::MatrixXd p_rho_in = p_raw;
        rho_->propagate_filter(p_rho_in);

        if (!kalman_->isReady()) {
            kalman_->prepareInitialCondition(t, q_raw, p_raw);
        } else {
            kalman_->prediction(t);
            kalman_->update(q_raw, p_raw);
        }

        // --- Baselines ---
        // 1. Numerical Differentiation (Backward Euler)
        Eigen::Vector3d v_numeric = (p_raw - last_pos_numeric_) / dt_;
        last_pos_numeric_ = p_raw;

        // 2. Dirty Derivative (Tustin)
        double tau = 1.0 / dirty_N_;
        double a1 = (2.0 * tau - dt_) / (2.0 * tau + dt_);
        double a2 = 2.0 / (2.0 * tau + dt_);
        
        dirty_vel_ = a1 * dirty_vel_ + a2 * (p_raw - last_pos_dirty_);
        last_pos_dirty_ = p_raw;

        // --- Transforms & Publishing ---
        // 1. Convert Raw (Y-up) -> ENU (Z-up)
        static const Eigen::Quaterniond q_w2enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        
        // Full orientation in ENU frame
        Eigen::Quaterniond q_enu = q_w2enu * q_raw;
        q_enu.normalize();

        // 2. Define the helper to publish
        auto publish_odom = [&](rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pub, 
                                const Eigen::Vector3d& p_est, 
                                const Eigen::Vector3d& v_est,
                                const Eigen::Vector3d& w_est = Eigen::Vector3d::Zero()) {
            
            nav_msgs::msg::Odometry odom = odom_msg_template_;
            odom.header.stamp = this->now(); 

            // Position: Local ENU Frame
            Eigen::Vector3d p_enu = q_w2enu * p_est;
            odom.pose.pose.position = tf2::toMsg(p_enu);
            odom.pose.pose.orientation = tf2::toMsg(q_enu); 

            // Velocity: Rotate ENU -> Full Body Frame (This matches BebopControlNode logic)
            Eigen::Vector3d v_enu = q_w2enu * v_est;
            Eigen::Vector3d v_body = q_enu.inverse() * v_enu;
            
            odom.twist.twist.linear.x = v_body.x();
            odom.twist.twist.linear.y = v_body.y();
            odom.twist.twist.linear.z = v_body.z();

            if (w_est.norm() > 1e-6) {
                Eigen::Vector3d w_enu = q_w2enu * w_est;
                Eigen::Vector3d w_body = q_enu.inverse() * w_enu;
                odom.twist.twist.angular.x = w_body.x();
                odom.twist.twist.angular.y = w_body.y();
                odom.twist.twist.angular.z = w_body.z();
            }

            pub->publish(odom);
        };

        publish_odom(pub_levant_, levant_->getPosition(), levant_->getVelocity());
        publish_odom(pub_rho_, rho_->get_position_estimate(), rho_->get_velocity_estimate());

        if (kalman_->isReady()) {
            publish_odom(pub_kalman_, kalman_->position, kalman_->linear_vel, kalman_->angular_vel);
        }

        publish_odom(pub_dirty_, p_raw, dirty_vel_);
        publish_odom(pub_numeric_, p_raw, v_numeric);
    }

    bool initialized_ = false;
    bool has_data_ = false;
    double dt_;
    Eigen::Vector3d dirty_vel_, last_pos_dirty_, last_pos_numeric_;
    double dirty_N_;
    nav_msgs::msg::Odometry odom_msg_template_;
    
    // Thread safety
    std::mutex data_mutex_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_msg_;

    std::unique_ptr<mocap_filters::LevantFilter> levant_;
    std::unique_ptr<mocap_filters::KalmanFilter> kalman_;
    std::unique_ptr<mocap_filters::RhoFilter> rho_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_levant_, pub_rho_, pub_kalman_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_dirty_, pub_numeric_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // MultiThreadedExecutor is safer if we want the sub and timer to truly operate in parallel,
    // though SingleThreaded works fine for this light load (callbacks queue up).
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<MocapFiltersNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}