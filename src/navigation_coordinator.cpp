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

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_state.pose.pose.orientation.z = -current_state.pose.pose.orientation.z;
    current_state.pose.pose.position.y = -current_state.pose.pose.position.y;
    current_pose.pose = current_state.pose.pose;
}

bool move2coord(float goal_pose_x, float goal_pose_y)
{
    bool success = true;
    TrajBuilder trajBuilder;
    mobot_controller::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;

    bool success_rotate;
    bool success_translate;

    // For now: rotate to head forward to goal point, then move toward the place.
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = goal_pose_x;
    double y_end = goal_pose_y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;

    double des_psi = atan2(dy, dx);

    ROS_INFO("Start_x = = %f", x_start);
    ROS_INFO("Start_y = = %f", y_start);
    ROS_INFO("Goal_x = %f", x_end);
    ROS_INFO("Goal_y = %f", y_end);
    // rotate
    goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                  current_pose.pose.position.y,
                                                  des_psi); // keep the same x,y, only rotate to des_psi
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = goal_pose_rot;
    srv.request.mode = "2"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_rotate = srv.response.success;
        ROS_INFO("rotate success? %d", success_rotate);
    }
    ros::spinOnce();

    // forward
    goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                    goal_pose_y,
                                                    des_psi); // keep des_psi, change x,y
    srv.request.start_pos = goal_pose_rot;
    srv.request.goal_pos = goal_pose_trans;
    srv.request.mode = "1"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_translate = srv.response.success;
        ROS_INFO("translate success? %d", success_translate);
    }
    ros::spinOnce();

    // if fail to forward
    if (!success_translate)
    {
        ROS_INFO("Cannot move, obstacle. braking");
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = current_pose; //anything is fine.
        srv.request.mode = "3";              // spin so that head toward the goal.
        client.call(srv);
        success = false;
    }
    ros::spinOnce();

    return success;
}

void tryMove(float goal_pose_x, float goal_pose_y, int retry_max)
{
    int retry_ctr = 0;
    bool success = move2coord(goal_pose_x, goal_pose_y);
    while (!success && retry_ctr < retry_max) {
        ROS_WARN("RETRY %d", retry_ctr);
        retry_ctr++;
        success = move2coord(goal_pose_x,goal_pose_y);
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

    float x_i = current_pose.pose.position.x;
    float y_i = current_pose.pose.position.y;

    ROS_INFO("STEP 1");
    tryMove(x_i + 5, y_i, 1);

    ROS_INFO("STEP 2");
    tryMove(x_i + 5, y_i -1, 1);

    ROS_INFO("STEP 3");
    tryMove(x_i, y_i -1, 1);

    ROS_INFO("STEP 4");
    tryMove(x_i, y_i, 1);

    // orientation fix: rotate after final translation.
    ros::spinOnce();
    float x_current = current_pose.pose.position.x;
    float y_current = current_pose.pose.position.y;
    ROS_INFO("STEP: Reorienting back to original pose");
    tryMove(x_current + 0.01, y_current, 1);

    // ROS_INFO("STEP 5");
    // tryMove(-8, current_pose.pose.position.y - 0.05, 1);

    ros::spin();

    return 0;
}