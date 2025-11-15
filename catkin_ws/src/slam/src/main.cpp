#include "ros/ros.h"
#include "slam/local_slam_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");
    LocalSlamNode node;
    ros::spin();
    return 0;
}