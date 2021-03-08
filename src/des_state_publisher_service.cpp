#include <ros/ros.h>
#include <traj_builder/traj_builder.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobot_controller/ServiceMsg.h>
#include <nav_msgs/Odometry.h>
#include <string.h>

using namespace std;

geometry_msgs::Twist g_halt_twist;
geometry_msgs::Twist g_forward_twist;
geometry_msgs::Twist g_spin_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

bool lidar_alarm = false;
int s = -1; // this is unreal. state start with 1 for forward.
//ros::Publisher des_state_pub;
//ros::ServiceServer des_state_service;

ros::Publisher des_state_pub;
ros::Publisher des_twist_pub;
ros::Publisher twist_pub;

ros::Subscriber lidar_sub;
ros::Subscriber current_state_sub;

nav_msgs::Odometry current_state;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_state.pose.pose.orientation.z = -current_state.pose.pose.orientation.z;
    current_state.pose.pose.position.y = -current_state.pose.pose.position.y;
}

bool desStateServiceCallBack(mobot_controller::ServiceMsgRequest &request,
                             mobot_controller::ServiceMsgResponse &response)
{
    bool success = false;

    // convert mode from string to int:
    // 0 - init (maybe don't need this?)
    // 1 - foward
    // 2 - spin-in-place
    // 3 - halt
    int mode = stoi(request.mode);
    s = mode;

    g_start_pose = request.start_pos;
    g_end_pose = request.goal_pos;

    double dt = 0.1;
    ros::Rate looprate(1 / dt);
    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(0.1);
    trajBuilder.set_accel_max(0.1);
    trajBuilder.set_omega_max(0.1*10);
    trajBuilder.set_speed_max(0.6);

    // calculate the desired state stream using traj_builder lib.
    nav_msgs::Odometry des_state;

    std::vector<nav_msgs::Odometry> vec_of_states;

    ROS_WARN("RECEIVED START_X =  %f", g_start_pose.pose.position.x);
    ROS_WARN("RECEIVED START_Y =  %f", g_start_pose.pose.position.y);
    ROS_WARN("RECEIVED GOAL_X =  %f", g_end_pose.pose.position.x);
    ROS_WARN("RECEIVED GOAL_Y =  %f", g_end_pose.pose.position.y);

    switch (s)
    {

    // GOING FORWARD
    case 1:
        ROS_INFO("GOING FORWARD");
        trajBuilder.build_travel_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm)
            {
                ROS_INFO("cannot move, obstalce");
                return response.success = false;;        // try this
            }
        }
        return response.success = true;

    // SPIN
    case 2:
        ROS_INFO("GOING SPIN");
        trajBuilder.build_spin_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        return response.success = true;

    // BRAKE - HALT!!!!!!!!
    case 3:
        ROS_INFO("BRAKEEEEEEEEE");
        trajBuilder.build_braking_traj(g_start_pose, current_state.twist.twist, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        return response.success = true;

    //* illegal input. for testing only
    case 4:
        ROS_ERROR("Please type valid mode number");
        return response.success = false;
        // just in case. don't know if need this.
    }

    return response.success;
}

void lidarCallback(const std_msgs::Bool &lidar_alarm_recv)
{
    lidar_alarm = lidar_alarm_recv.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "des_state_publisher_service");
    ros::NodeHandle n;

    ros::ServiceServer des_state_service = n.advertiseService("des_state_publisher_service", desStateServiceCallBack);

    des_state_pub = n.advertise<nav_msgs::Odometry>("/desired_state", 1);

    lidar_sub = n.subscribe("/lidar_alarm", 1, lidarCallback);
    current_state_sub = n.subscribe("/current_state", 1, currStateCallback);\

    ROS_INFO("Ready to publish des_state / des_twist");
    ros::spin();
    return 0;
}
