#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

float v_x = 0.0;
float v_y = 0.0;
float v_z = 0.0;
float w_x = 0.0;
float w_y = 0.0;
float w_z = 0.5;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_test");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_test = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    geometry_msgs::Twist twist;

    twist.linear.x = -v_x;
    twist.linear.y = v_y;
    twist.linear.z = v_z;
    twist.angular.x = w_x;
    twist.angular.y = w_y;
    twist.angular.z = w_z;
    while (ros::ok()){
        cmd_vel_test.publish(twist);    
    ros::spinOnce();
    }
    return 0;
}
