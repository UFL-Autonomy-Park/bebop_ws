#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mocap_filters/KalmanFilter.hpp"

class KalmanNode : public rclcpp::Node {
public:
    KalmanNode() : Node("kalman_node") {
        auto input_topic = this->declare_parameter<std::string>("input_topic");
        auto output_topic = this->declare_parameter<std::string>("output_topic");
        parent_frame_ = this->declare_parameter<std::string>("parent_frame");
        child_frame_ = this->declare_parameter<std::string>("child_frame");

        Eigen::Matrix<double, 12, 12> u_cov = Eigen::Matrix<double, 12, 12>::Zero();
        u_cov.topLeftCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6; 
        u_cov.bottomRightCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * 0.5;
        
        filter_.init(u_cov, Eigen::Matrix<double, 6, 6>::Identity() * 1e-4, 120.0);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, rclcpp::SensorDataQoS(), std::bind(&KalmanNode::cb, this, std::placeholders::_1));
    }

private:
    void cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        Eigen::Vector3d p; Eigen::Quaterniond q;
        tf2::fromMsg(msg->pose.position, p);
        tf2::fromMsg(msg->pose.orientation, q);

        if (!filter_.isReady()) {
            filter_.prepareInitialCondition(t, q, p);
            return;
        }

        filter_.prediction(t);
        filter_.update(q, p);

        // Transform World(Y-up) -> ENU
        static const Eigen::Quaterniond q_w2enu(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond q_enu = q_w2enu * filter_.attitude;
        Eigen::Vector3d p_enu = q_w2enu * filter_.position;
        Eigen::Vector3d v_enu = q_w2enu * filter_.linear_vel;
        Eigen::Vector3d w_enu = q_w2enu * filter_.angular_vel;

        double r, pi, y;
        tf2::Matrix3x3(tf2::Quaternion(q_enu.x(), q_enu.y(), q_enu.z(), q_enu.w())).getRPY(r, pi, y);
        Eigen::Quaterniond q_yaw_inv(Eigen::AngleAxisd(-y, Eigen::Vector3d::UnitZ()));

        // --- DEFINE MISSING VARIABLES ---
        Eigen::Vector3d v_final = q_yaw_inv * v_enu;
        Eigen::Vector3d w_final = q_yaw_inv * w_enu;

        nav_msgs::msg::Odometry odom;
        odom.header = msg->header;
        odom.header.frame_id = parent_frame_;
        odom.child_frame_id = child_frame_;
        odom.pose.pose.position = tf2::toMsg(p_enu);
        odom.pose.pose.orientation = tf2::toMsg(q_enu);

        odom.twist.twist.linear.x = v_final.x();
        odom.twist.twist.linear.y = v_final.y();
        odom.twist.twist.linear.z = v_final.z();

        odom.twist.twist.angular.x = w_final.x();
        odom.twist.twist.angular.y = w_final.y();
        odom.twist.twist.angular.z = w_final.z();

        odom_pub_->publish(odom);
    }

    mocap_filters::KalmanFilter filter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::string parent_frame_, child_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanNode>());
    rclcpp::shutdown();
    return 0;
}