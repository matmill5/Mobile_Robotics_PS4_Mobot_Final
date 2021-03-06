// to be upgraded to perform land drift correction
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

ros::Publisher twist_pub;
ros::Subscriber des_state_sub;
ros::Subscriber cur_state_sub;

nav_msgs::Odometry cur_state;
nav_msgs::Odometry des_state;

geometry_msgs::Twist cur_twist;
geometry_msgs::Twist des_twist;

// This just to update the current state with its twist
void curStateCallback(const nav_msgs::Odometry& cur_state_recv) {
    cur_state = cur_state_recv;
}

void desStateCallback(const nav_msgs::Odometry& des_state_recv){
    des_state = des_state_recv;
    des_twist = des_state.twist.twist;
    //! add logic here to distinguise either spin-in-place, straight-line-motion, or halt
    //! after logic, lane-drift control!!!!
    twist_pub.publish(des_twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "modal_trajectory_controller"); //name this node
    ros::NodeHandle nh; 

    twist_pub = nh.advertise<geometry_msgs::Twist>("lidar_alarm", 1);
    
    des_state_sub = nh.subscribe("/desired_state", 1, desStateCallback);
    cur_state_sub = nh.subscribe("/current_state", 1, curStateCallback);

    ros::spin();    
    return 0; 
}
