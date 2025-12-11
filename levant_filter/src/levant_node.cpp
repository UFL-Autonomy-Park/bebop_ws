#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "Eigen/Geometry"
#include "levant_filter/LevantFilter.hpp"

using namespace std::chrono_literals;

class LevantNode : public rclcpp::Node {
public:
    LevantNode() : Node("levant_node"), initialized_(false) {
        double C = this->declare_parameter<double>("lipschitz_constant");
        int filter_period_ms = this->declare_parameter<int>("filter_period_ms");
        parent_frame_ = this->declare_parameter<std::string>("parent_frame", "world");
        child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");

        dt_ = filter_period_ms / 1000.0;
        filter_ = std::make_unique<LevantFilter>(C, dt_);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("levant_filtered_odom", 10);
        
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn_mocap/bebop104/pose", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                last_pose_ = *msg;
                initialized_ = true;
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(filter_period_ms), 
            std::bind(&LevantNode::update, this));
    }

private:
    void update() {
        if (!initialized_) return;

        Eigen::Vector3d raw_pos(
            last_pose_.pose.position.x,
            last_pose_.pose.position.y,
            last_pose_.pose.position.z
        );

        filter_->propagate(raw_pos);
        
        Eigen::Vector3d p_est = filter_->getPosition();
        Eigen::Vector3d v_est = filter_->getVelocity();

        // Transforms: World(Y-up) -> ENU(Z-up)
        // Static rotation +90 deg about X
        static const Eigen::Quaterniond q_w2enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

        Eigen::Vector3d p_enu = q_w2enu * p_est;
        Eigen::Vector3d v_enu = q_w2enu * v_est;

        Eigen::Quaterniond q_raw(
            last_pose_.pose.orientation.w,
            last_pose_.pose.orientation.x,
            last_pose_.pose.orientation.y,
            last_pose_.pose.orientation.z
        );
        Eigen::Quaterniond q_enu = q_w2enu * q_raw;

        // Extract Yaw for Body Velocity
        double roll, pitch, yaw;
        tf2::Quaternion tf_q(q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w());
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        Eigen::Quaterniond q_yaw_inv(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d v_body = q_yaw_inv * v_enu;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = parent_frame_;
        odom.child_frame_id = child_frame_;

        odom.pose.pose.position.x = p_enu.x();
        odom.pose.pose.position.y = p_enu.y();
        odom.pose.pose.position.z = p_enu.z();
        odom.pose.pose.orientation = tf2::toMsg(q_enu);

        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();

        odom_pub_->publish(odom);
    }

    bool initialized_;
    double dt_;
    std::string parent_frame_, child_frame_;
    geometry_msgs::msg::PoseStamped last_pose_;
    std::unique_ptr<LevantFilter> filter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LevantNode>());
    rclcpp::shutdown();
    return 0;
}