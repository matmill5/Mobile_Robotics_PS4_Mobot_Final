#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <nav_msgs/Odometry.h>

ros::Subscriber odom_subscriber;

ros::Publisher current_state_publisher;

nav_msgs::Odometry odom;

void odomCallback (const nav_msgs::Odometry& odomReceived) {
    odom = odomReceived;
    current_state_publisher.publish(odom);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom.pose.pose.position.x,odom.pose.pose.position.y, odom.pose.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom.twist.twist.linear.x,odom.twist.twist.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc,argv, "current_state_publisher"); // name of the node
    ros::NodeHandle nh;

    current_state_publisher = nh.advertise<nav_msgs::Odometry>("/current_state",1);

    odom_subscriber = nh.subscribe("odom", 1, odomCallback);

    ros::spin();
    return 0;
}