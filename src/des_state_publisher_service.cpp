#include <ros/ros.h>
#include <traj_builder/traj_builder.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobot_controller/ServiceMsg.h>
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

bool desStateServiceCallBack(mobot_controller::ServiceMsgRequest &request,
                             mobot_controller::ServiceMsgResponse &response)
{
    bool success = true;

    // convert mode from string to int:
    // 0 - init (maybe don't need this?)
    // 1 - foward
    // 2 - spin-in-place
    // 3 - halt
    int mode = stoi(request.mode);
    s = mode;

    g_start_pose = request.start_pos;
    g_end_pose = request.goal_pos;

    double dt = 0.01;
    ros::Rate looprate(1 / dt);
    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    // trajBuilder.set_alpha_max(1.0);
    // trajBuilder.set_accel_max(1.0);

    // calculate the desired state stream using traj_builder lib.
    nav_msgs::Odometry des_state;

    std::vector<nav_msgs::Odometry> vec_of_states;

    switch (s)
    {
    // case 0:
    //     trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);
    //     for (auto state : vec_of_states)
    //     {
    //         des_state = state;
    //         des_state.header.stamp = ros::Time::now();
    //         des_state_pub.publish(des_state);
    //         looprate.sleep();
    //         ros::spinOnce();
    //         if (lidar_alarm)
    //             response.success = false;
    //     }
    //     break;

    // GOING FORWARD
    case 1:
        trajBuilder.build_travel_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm){
                s = 3;
                response.success = false;
                break;
            }
        }
        break;

    // SPIN
    case 2:
        trajBuilder.build_spin_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm){
                s = 3;
                response.success = false;
                break;
            }
        }
        break;

    // BRAKE - HALT!!!!!!!!
    case 3:
        ROS_INFO("BRAKEEEEEEEEE");
        trajBuilder.build_braking_traj(g_start_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        break;

    //* illegal input. for testing only
    case 4:
        ROS_ERROR("Please type valid mode number");
        response.success = false;
        break; // just in case. don't know if need this.
    }

    ROS_INFO("Is movement successful?: %d", success);

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

    lidar_sub = n.subscribe("lidar_alarm", 1, lidarCallback);

    ros::spin();
    return 0;
}
