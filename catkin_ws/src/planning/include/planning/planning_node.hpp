#pragma once

#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"

#include <thread>
#include <mutex>

class PathPlanningNode 
{
public:
    PathPlanningNode();
    ~PathPlanningNode();

    // --- threads ---

    void planning_thread();

    // --- ros callbacks ---

    void map_slam_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // --- thread safe getters / setters ---
    
    nav_msgs::OccupancyGrid get_map();
    void set_map(const nav_msgs::OccupancyGrid& map);

private:
    // --- ros ---
    
    ros::NodeHandle _nh;

    ros::Subscriber _map_slam_sub;

    // --- threads ---

    std::thread _planning_thread_handle;

    // --- path planning internal state ---

    // input
    std::mutex _input_mutex;
    nav_msgs::OccupancyGrid _map;           // dynamic probability map from SLAM
};