#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <vector>
#include <math.h>

// these values to be set within the laser callback
int setup = -1; // NOT real; callback will have to find this
double angle_min_=0.0;              // -1.570799 or -pi/2
double angle_max_=0.0;              // 1.570799 or pi/2
double angle_increment_=0.0;        // 0.0174533333
double range_min_ = 0.0;            // will be 0.1
double range_max_ = 0.0;            // will be 80.0
bool laser_alarm_= false;
int centerIndex = -1; //! NOT real. Update within callback

// front edge info:
// the width can be choose to be bigger than the distance between two fron wheels:
// left_wheel_y = 0.282573
// right_wheel_y = -0.282873
const float width = 0.8;            // half the width of the rectangular box
const float dist_detect = 2.5;     // distance to closer edge (there should be a min value for this)

std::vector<float> range_limit;


int left_far_index, right_far_index;
double left_near_angle, right_near_angle;
double left_far_angle, right_far_angle;

ros::Publisher lidar_alarm_pub;

// Helper functions
// Convert angle to index for the laser scan
int angle2Index(double angle) {
    int index = (angle - angle_min_)/angle_increment_;
    return index;
}

// Convert index to angle for the laser scan
double index2Angle(int index) {
    double angle = index * angle_increment_ + angle_min_;
    return angle;
}

// Slicing the vector of the wanted windows
std::vector<float> vecSlice(std::vector<float> &v, int m, int n)    {
    std::vector<float> vec;
    for (int i=m; i<n+1; i++) 
        vec.push_back(v[i]); 
    return vec;
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    // setup the call back function for the first message.
    if (setup < 1)  {
        // Lidar's params
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

        // Box's params
        //! make sure that the index within the set range.
        right_far_angle = atan2(-width/2, dist_detect);
        left_far_angle = atan2(width/2, dist_detect);
        right_far_index = angle2Index(right_far_angle);
        left_far_index = angle2Index(left_far_angle);
        
        // Generate a box limit for range:

        // vector of ranges to the point within the box
        std::vector<float> range_limit_holder(laser_scan.ranges.size());
        for (int i=90; i < range_limit_holder.size(); i++){
            range_limit_holder[i] = 0;             // reset vector value
            float angle = index2Angle(i);
            // this is the right edge
            if (i < right_far_index)
                range_limit_holder[i] = width/2/cos(M_PI/2 + angle);
            // this is the left edge
            else if (i > left_far_index)
                range_limit_holder[i] = width/2/cos(M_PI/2 - angle);       
            // this is the middle part
            else
                range_limit_holder[i] = dist_detect/cos(angle);
        }

        //! BUGGGGGGGG
        // range_limit_holder[88] = range_limit_holder[92];
        // range_limit_holder[87] = range_limit_holder[93];
        // for (int i = 0; i<90; i++){
        //     range_limit_holder[i] = range_limit_holder[180-i];
        // }

        range_limit = range_limit_holder;
        // Complete setting up
        setup++;   
    }

    laser_alarm_ = false;
    for (int i = 0; i < laser_scan.ranges.size(); i++) {
        // ROS_INFO("range %f", laser_scan.ranges[i]);
        // ROS_INFO("limit %f", range_limit[i]);
        // ROS_INFO("angle: %d", i);
        if (laser_scan.ranges[i] <= range_limit[i]){
            laser_alarm_ = true;
            ROS_INFO("OBSTACLE!!!!!!!!!!");
        }
    }

    // Report metrics to us via the terminal
    ROS_INFO("Lidar Alarm = %d", laser_alarm_);
    // ROS_INFO("right_far = %f", laser_scan.ranges[right_far_index]);
    // ROS_INFO("right_th = %f", range_limit[right_far_index]);
    // ROS_INFO("lef_far = %f", laser_scan.ranges[left_far_index]);
    // ROS_INFO("left_th = %f", range_limit[left_far_index]);
    // Construct message and publish it
    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_pub.publish(lidar_alarm_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm_mobot"); //name this node
    ros::NodeHandle nh; 

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("/lidar_alarm", 1);
    lidar_alarm_pub = pub;
    
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
    
    ros::spin();    
    return 0; 
}

