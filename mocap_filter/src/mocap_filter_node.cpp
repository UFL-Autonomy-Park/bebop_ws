#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mocap_filter/KalmanFilter.h"
#include "tf2_eigen/tf2_eigen.hpp"

using namespace std;
using namespace std::placeholders;

class MocapFilterNode : public rclcpp::Node {
public:
    MocapFilterNode(): Node("mocap_filter_node") {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "/vrpn_mocap/bebop104/pose");
        this->declare_parameter<std::string>("output_topic", "/bebop104/filtered_odom");
        this->declare_parameter<std::string>("parent_frame", "/world");
        this->declare_parameter<std::string>("child_frame", "/bebop104/base_link");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        // Initialize Kalman Filter
        Eigen::Matrix<double, 12, 12> u_cov = Eigen::Matrix<double, 12, 12>::Zero();
        Eigen::Matrix<double, 6, 6> m_cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        // Process noise
        u_cov.topLeftCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6; 
        u_cov.bottomRightCorner<6,6>() = Eigen::Matrix<double, 6, 6>::Identity() * 0.5;

        kFilter_.init(u_cov, m_cov, 120);

        // Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            output_topic, 10);

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, rclcpp::SensorDataQoS(), 
            std::bind(&MocapFilterNode::poseCallback, this, _1));
        
        // Log
        RCLCPP_INFO(this->get_logger(), "Listening to: %s", input_topic.c_str());
    }
private:
    void poseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    ) {
        double time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Convert ROS to Eigen
        Eigen::Vector3d m_pos(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
        Eigen::Quaterniond m_att(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);
        if (!kFilter_.isReady()) {
            kFilter_.prepareInitialCondition(time, m_att, m_pos);
            return;
        }

        // Filter step
        kFilter_.prediction(time);
        kFilter_.update(m_att, m_pos);

        // Coordinate transform
        // world -> body
        // y is up in world, z is up in body
        Eigen::Matrix3d R_world_to_enu; 
        double theta = M_PI / 2.0;
        double c = std::cos(theta);
        double s = std::sin(theta);
        R_world_to_enu << 1,  0,  0,
                          0,  c, -s,
                          0,  s,  c;
        Eigen::Quaterniond q_world_to_enu(R_world_to_enu);
        Eigen::Quaterniond q_att_enu = q_world_to_enu * m_att;
        // Extract yaw
        double roll, pitch, yaw;
        tf2::Quaternion tf_q_enu(
            q_att_enu.x(), q_att_enu.y(), q_att_enu.z(), q_att_enu.w()
        );
        tf2::Matrix3x3(tf_q_enu).getRPY(roll, pitch, yaw);

        // Convert vels to ENU
        Eigen::Vector3d linear_vel_enu = R_world_to_enu * kFilter_.linear_vel;
        Eigen::Vector3d angular_vel_enu = R_world_to_enu * kFilter_.angular_vel;

        // Remove yaw to get body frame
        Eigen::Quaterniond q_yaw_correction(
            Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ())
        );
        // Convert vels to body frame
        Eigen::Vector3d linear_vel_body = q_yaw_correction * linear_vel_enu;
        Eigen::Vector3d angular_vel_body = q_yaw_correction * angular_vel_enu;

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = parent_frame_;
        odom_msg.child_frame_id = child_frame_;

        // Eigen to ROS
        odom_msg.pose.pose.position.x = kFilter_.position.x();
        odom_msg.pose.pose.position.y = kFilter_.position.y();
        odom_msg.pose.pose.position.z = kFilter_.position.z();
        odom_msg.pose.pose.orientation.w = kFilter_.attitude.w();
        odom_msg.pose.pose.orientation.x = kFilter_.attitude.x();
        odom_msg.pose.pose.orientation.y = kFilter_.attitude.y();
        odom_msg.pose.pose.orientation.z = kFilter_.attitude.z();

        odom_msg.twist.twist.linear.x = linear_vel_body.x();
        odom_msg.twist.twist.linear.y = linear_vel_body.y();
        odom_msg.twist.twist.linear.z = linear_vel_body.z();
        odom_msg.twist.twist.angular.x = angular_vel_body.x();
        odom_msg.twist.twist.angular.y = angular_vel_body.y();
        odom_msg.twist.twist.angular.z = angular_vel_body.z();

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    mocap::KalmanFilter kFilter_;
    std::string parent_frame_, child_frame_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapFilterNode>());
    rclcpp::shutdown();
    return 0;
}