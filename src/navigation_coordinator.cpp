#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mobot_controller/ServiceMsg.h>
#include <traj_builder/traj_builder.h>

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
    current_state.pose.pose.orientation.z = -current_state.pose.pose.orientation.z;
    current_state.pose.pose.position.y = - current_state.pose.pose.position.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    vector<geometry_msgs::PoseStamped> plan_points;

    ros::ServiceClient client = n.serviceClient<mobot_controller::ServiceMsg>("des_state_publisher_service");
    
    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    mobot_controller::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;

    TrajBuilder trajBuilder;
    double goal_pose_x = 0.0;
    double goal_pose_y = 0.0;
    double goal_pose_psi = 0.0;

    while (ros::ok()) {
        cout << "Enter a x: ";
    	cin  >> goal_pose_x;
        cout << "Enter a y: ";
        cin >> goal_pose_y;
        
        start_pose.pose = current_state.pose.pose;

        // For now: rotate to head forward to goal point, then move toward the place.
        double x_start = start_pose.pose.position.x;
        double y_start = start_pose.pose.position.y;
        double x_end = goal_pose_x;
        double y_end = goal_pose_y;
        double dx = x_end - x_start;
        double dy = y_end - y_start;
        
        double des_psi = atan2(dy,dx);

        // rotate
        while (fabs(trajBuilder.convertPlanarQuat2Psi(current_state.pose.pose.orientation)-des_psi)>= M_PI/18) {
            geometry_msgs::PoseStamped current_pose;
            current_pose.pose = current_state.pose.pose;
            goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                            current_pose.pose.position.y,
                                                                des_psi); // keep the same x,y, only rotate to des_psi
            srv.request.start_pos = current_pose; 
            srv.request.goal_pos = goal_pose_rot;
            srv.request.mode = "2";       // spin so that head toward the goal.
            client.call(srv);

            ros::spinOnce();
        }
        

        ROS_INFO("mid_pose_x: %f", goal_pose_rot.pose.position.x);
        ROS_INFO("mid_pose_y: %f", goal_pose_rot.pose.position.y);
        ROS_INFO("mid_pose_p: %f", des_psi);

        // forward
        goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                        goal_pose_y,
                                                         des_psi); // keep des_psi, change x,y
        srv.request.start_pos = goal_pose_rot; 
        srv.request.goal_pos = goal_pose_trans;
        srv.request.mode = "1";       // spin so that head toward the goal.
        client.call(srv);

        ROS_INFO("end_pose_x: %f", goal_pose_trans.pose.position.x);
        ROS_INFO("end_pose_y: %f", goal_pose_trans.pose.position.y);
        ROS_INFO("end_pose_p: %f", des_psi);

        ros::spinOnce();
    }

    return 0;
}