#include "ros/ros.h"
#include "planning/planning_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    PathPlanningNode node;
    ros::spin();
    return 0;
}