#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mobot_controller/ServiceMsg.h>
using namespace std;

nav_msgs::Odometry current_state;

//example ROS client:
// first run: rosrun stdr_wall_following_controller stdr_wall_following_controller
// then start this node:  rosrun stdr_wall_following_controller heading_ros_client

//given a heading for motion on a plane (as above), convert this to a quaternion
geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

void currStateCallback(const nav_msgs::Odometry &odom){
    current_state = odom;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    vector<geometry_msgs::PoseStamped> plan_points;

    ros::ServiceClient client = n.serviceClient<mobot_controller::ServiceMsg>("des_state_publisher_service");
    
    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    mobot_controller::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose;
    string mode;

    float goal_pose_x;
    float goal_pose_y;
    float goal_pose_theta;

    while (ros::ok()) {
        cout << "Enter a x: ";
    	cin  >> goal_pose_x;
        cout << "Enter a y: ";
        cin >> goal_pose_y;
        cout << "Enter a theta: ";
        cin >> goal_pose_theta;
        cout << "Enter a mode: ";
        cin >> mode;

        
        //x,y,theata -> stampedPose
        start_pose.pose = current_state.pose.pose;
        start_pose.header.stamp = ros::Time::now();
        goal_pose = start_pose;                         // keep the z at where it was.

        goal_pose.pose.position.x = goal_pose_x;
        goal_pose.pose.position.y = goal_pose_y;
        goal_pose.pose.orientation = convertPlanarPsi2Quaternion(goal_pose_theta);
       
    	srv.request.start_pos = start_pose;
        srv.request.goal_pos = goal_pose;
        srv.request.mode = mode;
   	    client.call(srv);

        ros::spinOnce();
    }

    return 0;
}