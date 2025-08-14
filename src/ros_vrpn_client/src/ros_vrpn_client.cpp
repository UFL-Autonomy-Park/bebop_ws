/*
# Copyright (c) 2011, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,s
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
*/

//== This application listens for a rigid body named 'Tracker' on a remote machine
//== and publishes & tf it's position and orientation through ROS.

// ros2 run ros_vrpn_client ros_vrpn_client --ros-args -r __node:=bebop01 -p vrpn_ip:="192.168.1.202" -p my_int:=3883



#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdio.h>
#include <math.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

using namespace std::chrono_literals;
// Daman
//#include <LinearMath/btQuaternion.h>
//#include <tf/LinearMath/Quaternion.h>
// /Daman

void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t);

class TargetState{
    public:
        geometry_msgs::msg::TransformStamped target;
        geometry_msgs::msg::PoseStamped target_pose_stamped;
        geometry_msgs::msg::TransformStamped ned_target;
        geometry_msgs::msg::PoseStamped ned_pose_stamped;
};

TargetState *target_state;
std::string frame_id;

// set to true in the VRPN callback function.
bool fresh_data = false;
vrpn_TRACKERCB prev_vrpn_data;


class Rigid_Body: public rclcpp::Node {
    private:
        //initialize publishers
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr target_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamp_pub;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ned_target_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ned_pose_stamp_pub;
        // Initialize the transform broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;
        vrpn_Connection *connection;
        vrpn_Tracker_Remote *tracker;
        rclcpp::TimerBase::SharedPtr timer_;
        //parameters

    public:
        Rigid_Body(): Node("ros_vrpn_client")
        {
            this->declare_parameter("vrpn_ip", std::string());
            this->declare_parameter("port", 3883);

            br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            target_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("~/nwu/pose", 100);
            pose_stamp_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/nwu/pose_stamped",100);

            ned_target_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("~/ned/pose", 100);
            ned_pose_stamp_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/ned/pose_stamped",100);

            std::string connec_nm = this->get_parameter("vrpn_ip").as_string() + ":" + std::to_string(this->get_parameter("port").as_int());
            connection = vrpn_get_connection_by_name(connec_nm.c_str());
            std::string target_name = this->get_name();
            RCLCPP_WARN_STREAM(this->get_logger(),"Connection informations: " << connec_nm);
            RCLCPP_WARN_STREAM(this->get_logger(),"Rigid body name: " << target_name);
            tracker = new vrpn_Tracker_Remote(target_name.c_str(), connection);
            this->tracker->register_change_handler(NULL, track_target);
            // Todo: tune mocap frequency
            timer_ = this->create_wall_timer(5ms, std::bind(&Rigid_Body::main_loop, this));
            }

        void publish_target_state(TargetState *target_state)
        {
            target_state->target.header.stamp= this->now();
            target_state->target_pose_stamped.header.stamp= this->now();
            target_state->ned_target.header.stamp= this->now();
            target_state->ned_pose_stamped.header.stamp= this->now();
            br->sendTransform(target_state->target);
            target_pub->publish(target_state->target);
            pose_stamp_pub->publish(target_state->target_pose_stamped);
            ned_target_pub->publish(target_state->ned_target);
            ned_pose_stamp_pub->publish(target_state->ned_pose_stamped);
        }

        void step_vrpn()
        {
            this->tracker->mainloop();
            this->connection->mainloop();
        }

        void main_loop() {
            step_vrpn();
            if (fresh_data == true)
            { // only publish when receive data over VRPN.
                publish_target_state(target_state);
                fresh_data = false;
            }
        }

};

//== Tracker Position/Orientation Callback ==--
void VRPN_CALLBACK track_target (void *, const vrpn_TRACKERCB t)
{
    // Daman
    //btQuaternion q_orig(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
    tf2::Quaternion q_orig(t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
    //btQuaternion q_fix(0.70710678, 0., 0., 0.70710678);
    tf2::Quaternion q_fix(0.70710678, 0., 0., 0.70710678);
    // /Daman

    // optitrak <-- funky <-- object
    // the q_fix.inverse() esures that when optitrak_funky says 0 0 0
    // for roll pitch yaw, there is still a rotation that aligns the
    // object frame with the /optitrak frame (and not /optitrak_funky)
    // Daman
    //btQuaternion q_rot = q_fix * q_orig * q_fix.inverse();
    tf2::Quaternion q_rot = q_fix * q_orig * q_fix.inverse();
    // /Daman

    //btScalar ang = q_rot.getAngle();
    // Daman
    //btVector3 axis = q_rot.getAxis();
//    tf2::Vector3 axis = q_rot.getAxis();
    //btVector3 pos(t.pos[0], -t.pos[2], t.pos[1]);
    tf2::Vector3 pos(t.pos[0], -t.pos[2], t.pos[1]);
    //btVector3 new_pos = pos.rotate(axis, ang);

    // verifying that each callback indeed gives fresh data.
    if ( prev_vrpn_data.quat[0] == t.quat[0] and \
         prev_vrpn_data.quat[1] == t.quat[1] and \
         prev_vrpn_data.quat[2] == t.quat[2] and \
         prev_vrpn_data.quat[3] == t.quat[3] and \
         prev_vrpn_data.pos[0] == t.pos[0] and \
         prev_vrpn_data.pos[1] == t.pos[1] and \
         prev_vrpn_data.pos[2] == t.pos[2] )
        std::cout <<"Repeated Values";

    prev_vrpn_data = t;

    target_state->target.transform.translation.x = pos.x();
    //target_state->target.transform.translation.x = pos.y();
    target_state->target.transform.translation.y = pos.y();
    //target_state->target.transform.translation.y = -pos.x();
    target_state->target.transform.translation.z = pos.z();

    target_state->target.transform.rotation.x = q_rot.x();
    target_state->target.transform.rotation.y = q_rot.y();
    target_state->target.transform.rotation.z = q_rot.z();
    target_state->target.transform.rotation.w = q_rot.w();

    target_state->target.header.frame_id = "optitrack";
    target_state->target.child_frame_id = frame_id;
//    target_state->target.header.stamp = this->now();


    target_state->target_pose_stamped.pose.position.x = pos.x();
    target_state->target_pose_stamped.pose.position.y = pos.y();
    target_state->target_pose_stamped.pose.position.z = pos.z();

    target_state->target_pose_stamped.pose.orientation.x = q_rot.x();
    target_state->target_pose_stamped.pose.orientation.y = q_rot.y();
    target_state->target_pose_stamped.pose.orientation.z = q_rot.z();
    target_state->target_pose_stamped.pose.orientation.w = q_rot.w();

    target_state->target_pose_stamped.header.frame_id = "optitrack";
//    target_state->target_pose_stamped.header.stamp = this->now();

    //NED frame

    target_state->ned_target.transform.translation.x = pos.x();
    //target_state->target.transform.translation.x = pos.y();
    target_state->ned_target.transform.translation.y = -pos.y();
    //target_state->target.transform.translation.y = -pos.x();
    target_state->ned_target.transform.translation.z = -pos.z();

    target_state->ned_target.transform.rotation.x = q_rot.x();
    target_state->ned_target.transform.rotation.y = -q_rot.y();
    target_state->ned_target.transform.rotation.z = -q_rot.z();
    target_state->ned_target.transform.rotation.w = q_rot.w();

    target_state->ned_target.header.frame_id = "optitrack";
    target_state->ned_target.child_frame_id = frame_id;
//    target_state->ned_target.header.stamp = this->now();



    target_state->ned_pose_stamped.pose.position.x = pos.x();
    target_state->ned_pose_stamped.pose.position.y = -pos.y();
    target_state->ned_pose_stamped.pose.position.z = -pos.z();

    target_state->ned_pose_stamped.pose.orientation.x = q_rot.x();
    target_state->ned_pose_stamped.pose.orientation.y = -q_rot.y();
    target_state->ned_pose_stamped.pose.orientation.z = -q_rot.z();
    target_state->ned_pose_stamped.pose.orientation.w = q_rot.w();

    target_state->ned_pose_stamped.header.frame_id = "optitrack";
//    target_state->ned_pose_stamped.header.stamp = this->now();


//        std::cout<<"marker x"<<pos.x()<<std::endl;
//        std::cout<<"marker y"<<pos.y()<<std::endl;
//        std::cout<<"marker z"<<pos.z()<<std::endl;

    fresh_data = true;
}


int main(int argc, char* argv[])
{

    target_state = new TargetState;

//    std::string vrpn_server_ip;
//    int vrpn_port;
//    std::string tracked_object_name;
//
//    nh.param<std::string>("vrpn_server_ip", vrpn_server_ip, std::string());
//    nh.param<int>("vrpn_port", vrpn_port, 3883);
//
//    std::cout<<"vrpn_server_ip:"<<vrpn_server_ip<<std::endl;
//    std::cout<<"vrpn_port:"<<vrpn_port<<std::endl;
//
//    Rigid_Body tool(nh, vrpn_server_ip, vrpn_port);

//    ros::Rate loop_rate(1000);
//
//    while(ros::ok())
//    {
//        tool.step_vrpn();
//        //vrpn_SleepMsecs(10);
//        if (fresh_data == true)
//        { // only publish when receive data over VRPN.
//            tool.publish_target_state(target_state);
//            fresh_data = false;
//        }
//        //ros::spinOnce();
//        loop_rate.sleep();
//    }
//	return 0;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Rigid_Body>());
    rclcpp::shutdown();
    return 0;
}
