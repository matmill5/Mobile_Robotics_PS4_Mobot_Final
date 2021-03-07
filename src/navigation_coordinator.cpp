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
geometry_msgs::PoseStamped current_pose;

ros::ServiceClient client;
//example ROS client:
// first run: rosrun stdr_wall_following_controller stdr_wall_following_controller
// then start this node:  rosrun stdr_wall_following_controller heading_ros_client

//given a heading for motion on a plane (as above), convert this to a quaternion
geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi)
{
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_state.pose.pose.orientation.z = -current_state.pose.pose.orientation.z;
    current_state.pose.pose.position.y = -current_state.pose.pose.position.y;
    current_pose.pose = current_state.pose.pose;
}

void move2coord(float goal_pose_x, float goal_pose_y)
{
    TrajBuilder trajBuilder;
    mobot_controller::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;

    // For now: rotate to head forward to goal point, then move toward the place.
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = goal_pose_x;
    double y_end = goal_pose_y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;

    double des_psi = atan2(dy, dx);

    // rotate
    while (fabs(trajBuilder.convertPlanarQuat2Psi(current_state.pose.pose.orientation) - des_psi) >= M_PI / 18)
    {
        goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                      current_pose.pose.position.y,
                                                      des_psi); // keep the same x,y, only rotate to des_psi
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = goal_pose_rot;
        srv.request.mode = "2"; // spin so that head toward the goal.
        client.call(srv);
        ros::spinOnce();
    }

    // forward
    goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                    goal_pose_y,
                                                    des_psi); // keep des_psi, change x,y
    srv.request.start_pos = goal_pose_rot;
    srv.request.goal_pos = goal_pose_trans;
    srv.request.mode = "1"; // spin so that head toward the goal.
    client.call(srv);
    ros::spinOnce();

    // if fail to forward
    if (!srv.response.success)
    {
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = current_pose; //anything is fine.
        srv.request.mode = "3";              // spin so that head toward the goal.
        client.call(srv);
        ROS_INFO("cannot move, obstacle. braking");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    vector<geometry_msgs::PoseStamped> plan_points;

    client = n.serviceClient<mobot_controller::ServiceMsg>("des_state_publisher_service");

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    TrajBuilder trajBuilder;

    ROS_INFO("STEP 1");
    move2coord(10,0);

    ROS_INFO("STEP 2");
    move2coord(current_pose.pose.position.x - 0.05, 10);

    ROS_INFO("STEP 3");
    move2coord(0, current_pose.pose.position.y - 0.05);

    ROS_INFO("STEP 4");
    move2coord(0, 7.5);

    ROS_INFO("STEP 5");
    move2coord(-8, 7.5);
    
    ros::spin();

    return 0;
}