#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mocap_filters/RhoFilter.hpp"

class RhoNode : public rclcpp::Node {
public:
    RhoNode() : Node("rho_node"), initialized_(false) {
        double k1 = this->declare_parameter<double>("k1");
        double k2 = this->declare_parameter<double>("k2");
        double k3 = this->declare_parameter<double>("k3");
        double alpha = this->declare_parameter<double>("alpha");
        int ms = this->declare_parameter<int>("filter_period_ms");
        parent_frame_ = this->declare_parameter<std::string>("parent_frame");
        child_frame_ = this->declare_parameter<std::string>("child_frame");

        // Namespace now exists, so this works
        filter_ = std::make_unique<mocap_filters::RhoFilter>(ms / 1000.0, 3, alpha, k1, k2, k3);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("rho_filtered_odom", 10);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn_mocap/bebop104/pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                last_pose_ = *msg;
                initialized_ = true;
            });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(ms), [this](){ publish(); });
    }

private:
    void publish() {
        if (!initialized_) return;
        
        Eigen::Vector3d p_raw_vec;
        tf2::fromMsg(last_pose_.pose.position, p_raw_vec);
        Eigen::MatrixXd p_raw_mat = p_raw_vec; 
        filter_->propagate_filter(p_raw_mat);

        Eigen::Vector3d p_est = filter_->get_position_estimate();
        Eigen::Vector3d v_est = filter_->get_velocity_estimate();

        // Transform World(Y-up) -> ENU(Z-up)
        static const Eigen::Quaterniond q_w2enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        
        Eigen::Vector3d p_enu = q_w2enu * p_est;
        Eigen::Vector3d v_enu = q_w2enu * v_est;

        Eigen::Quaterniond q_raw;
        tf2::fromMsg(last_pose_.pose.orientation, q_raw);
        Eigen::Quaterniond q_enu = q_w2enu * q_raw;

        double r, pi, y;
        tf2::Matrix3x3(tf2::Quaternion(q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w())).getRPY(r, pi, y);
        
        // Remove Yaw from velocity
        Eigen::Vector3d v_body = Eigen::AngleAxisd(-y, Eigen::Vector3d::UnitZ()) * v_enu;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = parent_frame_;
        odom.child_frame_id = child_frame_;
        odom.pose.pose.position = tf2::toMsg(p_enu);
        odom.pose.pose.orientation = tf2::toMsg(q_enu);
        
        // Manual assignment
        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();
        
        odom_pub_->publish(odom);
    }

    bool initialized_;
    std::string parent_frame_, child_frame_;
    geometry_msgs::msg::PoseStamped last_pose_;
    std::unique_ptr<mocap_filters::RhoFilter> filter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RhoNode>());
    rclcpp::shutdown();
    return 0;
}