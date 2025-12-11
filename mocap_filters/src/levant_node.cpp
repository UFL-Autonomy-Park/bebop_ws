#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mocap_filters/LevantFilter.hpp"

class LevantNode : public rclcpp::Node {
public:
    LevantNode() : Node("levant_node"), initialized_(false) {
        double C = this->declare_parameter<double>("lipschitz_constant");
        int ms = this->declare_parameter<int>("filter_period_ms");
        parent_frame_ = this->declare_parameter<std::string>("parent_frame");
        child_frame_ = this->declare_parameter<std::string>("child_frame");

        filter_ = std::make_unique<mocap_filters::LevantFilter>(C, ms / 1000.0);
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("levant_filtered_odom", 10);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn_mocap/bebop104/pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                last_pose_ = *msg;
                initialized_ = true;
            });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(ms), [this](){ update(); });
    }

private:
    void update() {
        if (!initialized_) return;
        
        Eigen::Vector3d p_raw(last_pose_.pose.position.x, last_pose_.pose.position.y, last_pose_.pose.position.z);
        filter_->propagate(p_raw);

        // Rotation: World(Y-up) -> ENU(Z-up)
        static const Eigen::Quaterniond q_w2enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        
        Eigen::Vector3d p_enu = q_w2enu * filter_->getPosition();
        Eigen::Vector3d v_enu = q_w2enu * filter_->getVelocity();

        Eigen::Quaterniond q_raw;
        tf2::fromMsg(last_pose_.pose.orientation, q_raw);
        Eigen::Quaterniond q_enu = q_w2enu * q_raw;

        // Yaw Extraction for Body Velocity
        double r, p, y;
        tf2::Matrix3x3(tf2::Quaternion(q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w())).getRPY(r, p, y);
        Eigen::Vector3d v_body = Eigen::AngleAxisd(-y, Eigen::Vector3d::UnitZ()) * v_enu;

        nav_msgs::msg::Odometry odom;
        // ... headers ...
        odom.pose.pose.position = tf2::toMsg(p_enu);
        odom.pose.pose.orientation = tf2::toMsg(q_enu);

        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();

        odom_pub_->publish(odom);
    }

    bool initialized_;
    std::string parent_frame_, child_frame_;
    geometry_msgs::msg::PoseStamped last_pose_;
    std::unique_ptr<mocap_filters::LevantFilter> filter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LevantNode>());
    rclcpp::shutdown();
    return 0;
}