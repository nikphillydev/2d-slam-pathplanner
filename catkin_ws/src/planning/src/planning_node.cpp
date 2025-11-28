#include "planning/planning_node.hpp"

PathPlanningNode::PathPlanningNode()
{
    // create subscribers
    _map_slam_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("/map/slam", 1000, &PathPlanningNode::map_slam_callback, this);

    // start worker threads
    _planning_thread_handle = std::thread(&PathPlanningNode::planning_thread, this);

    ROS_INFO("PathPlanningNode has started");
}

PathPlanningNode::~PathPlanningNode()
{
    if (_planning_thread_handle.joinable()) _planning_thread_handle.join();

    ROS_INFO("PathPlanningNode has stopped");
}

// --- worker threads ---

void PathPlanningNode::planning_thread()
{
    ros::Rate loop_rate(2);
    
    while(ros::ok())
    {
        ROS_INFO("path planning running");

        loop_rate.sleep();
    }
}

// --- ros callbacks ---

void PathPlanningNode::map_slam_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("received map!");
    set_map(*msg);
}

// --- getters / setters ---

nav_msgs::OccupancyGrid PathPlanningNode::get_map()
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    return _map;
}

void PathPlanningNode::set_map(const nav_msgs::OccupancyGrid& map)
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    _map = map;
}