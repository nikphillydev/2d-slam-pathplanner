#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::stringstream ss;
    ss << "Received LaserScan message, frame_id: " << msg->header.frame_id
        << " and stamp " << msg->header.stamp.sec << std::endl;
    ROS_INFO("%s", ss.str().c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/front/scan", 1000, front_scan_callback);
    ros::spin();
    return 0;
}