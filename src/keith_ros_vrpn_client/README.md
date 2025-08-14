A ROS package to get data from the Optitrack motion capture system

Installation
------------

This installation assumes you have ROS installed and created a colcon workspace on your computer

1. In the command terminal, move to your catkin workspace's src directory (cd colcon_ws/src)

2. Once inside the catkin src directory type: git clone {this url repo}
	and enter in your GitLab credentials when prompted

3. Either move back out of your catkin src directory to use colcon build (ignore the warnings)

4. Once done you can begin to use vrpn node with motive and optitrack


Important notes before running
---------------

- Make sure that you got the right IP address of the computer that is running motive (in our case its 192.168.1.104)

-Make sure your computer is on the pathfinder or pathfinder5 network i.e same network as the optitrack system

-Have your robot's rigid body created and tracked in the motive/optitrack system

-By default the measurements come in the camera frame (ENU frame), this node automatically converts the messages to the  North East Down (NED) and North West Up (NWU) frames


-Location messages are in the form of:

	geometry_msg/TransformedStamped (robotrigidbody/ned or nwu/pose)

	and

	geometry_msg/PoseStampted (robotrigidbody/ned or nwu/pose_stamped)

	where robotrigidbody is the name of the robot's rigid body set in motive or optitrack.

	To see them when the vrpn client is active type: ros2 topic list


Running the code
----------------

	geometry_msg/TransformedStamped (/robotrigidbody/ned or nwu/pose)

-To run vrpn from the command line type: ros2 run ros_vrpn_client ros_vrpn_client --ros-args -r __node:=robotrigidbody -p vrpn_ip:="192.168.1.104" -p my_int:=3883
	where robotrigidbody is the name of the robot's rigid body set in motive or optitrack.

-To run vrpn from within a launch file add:

    <node pkg="ros_vrpn_client" exec="ros_vrpn_client" name="$(var name)">
        <param name="vrpn_ip" value="192.168.1.104"/>
        <param name="my_int" value="3883"/>
    </node>

	Please take a look at the example launch file
