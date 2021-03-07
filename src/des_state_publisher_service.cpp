#include <ros/ros.h>
#include <traj_builder/traj_builder.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mobot_controller/ServiceMsg.h>
#include <string.h>
geometry_msgs::Twist g_halt_twist;
geometry_msgs::Twist g_forward_twist;
geometry_msgs::Twist g_spin_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;
bool lidar_alarm = false;

//ros::Publisher des_state_pub;
//ros::ServiceServer des_state_service;

ros::Publisher des_state_pub;
ros::Publisher des_twist_pub;
ros::Publisher twist_pub;

ros::Subscriber lidar_sub;

using namespace std;

bool desStateServiceCallBack(mobot_controller::ServiceMsgRequest& request, mobot_controller::ServiceMsgResponse& response){
    bool success;
    int mode = stoi(request.mode);
    g_start_pose = request.start_pos;
    g_end_pose = request.goal_pos;
    
    double dt = 1;
    ros::Rate looprate(1/dt);    
    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    // trajBuilder.set_alpha_max(1.0);
    // trajBuilder.set_accel_max(1.0);

    nav_msgs::Odometry des_state;

    std::vector<nav_msgs::Odometry> vec_of_states;
    
    
    // Forward mode - 1
    if (mode == 1) {
        trajBuilder.build_travel_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state:vec_of_states) {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm)
                return response.success = false;                
        }
    }
    //Spin mode - 2
    if (mode == 2) {
        trajBuilder.build_spin_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state:vec_of_states) {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm)
                return response.success = false;    
        }
    }
    if(mode = 3){
        trajBuilder.build_braking_traj(g_start_pose, vec_of_states);
        for (auto state:vec_of_states) {
            des_state = state;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm)
                return response.success = false;    
        }
    }
    else {
        ROS_ERROR("Eh!!!!!!!");
        return response.success = false;    
    }
    return response.success = true;
}

void lidarCallback(const std_msgs::Bool& lidar_alarm_recv){
    lidar_alarm = lidar_alarm_recv.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher_service");
    ros::NodeHandle n;

    ros::ServiceServer des_state_service = n.advertiseService("des_state_publisher_service", desStateServiceCallBack);

    des_state_pub = n.advertise<nav_msgs::Odometry>("/desired_state",1);

    lidar_sub = n.subscribe("lidar_alarm", 1, lidarCallback);
    
    ros::spin();
    return 0;
}
