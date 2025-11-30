#pragma once

#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

#include "planning/AStar.hpp"
#include "planning/pure_pursuit.hpp"

#include <thread>
#include <mutex>

class PathPlanningNode 
{
public:
    PathPlanningNode();
    ~PathPlanningNode();

    // --- threads ---

    void planning_thread();
    void emergency_stop_thread();
    void controller_thread();

    // --- ros callbacks ---

    void map_slam_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg);

    // --- thread safe getters / setters ---
    
    nav_msgs::OccupancyGrid get_map();
    void set_map(const nav_msgs::OccupancyGrid& map);
    geometry_msgs::Point get_goal();
    void set_goal(const geometry_msgs::Point& goal);
    sensor_msgs::LaserScan get_laser_scan();
    void set_laser_scan(const sensor_msgs::LaserScan& scan);


private:
    // --- ros ---
    
    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    ros::Subscriber _map_slam_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _laser_scan_sub;
  
    ros::Publisher _path_pub;
    ros::Publisher _estop_pub;
    ros::Publisher _cmd_vel_pub;

    // --- data ---
    
    std::mutex _input_mutex;
    nav_msgs::OccupancyGrid _map;
    geometry_msgs::Point _current_goal;
    sensor_msgs::LaserScan _laser_scan;
    double EMERGENCY_STOP_DIS = 0.3;
    bool _has_goal;
    bool _has_map;

    // --- threads ---

    std::thread _planning_thread_handle;
    std::thread _emergency_stop_thread_handle;
    std::thread _controller_thread_handle;

    // --- helpers ---
  
    void publish_path(const std::vector<geometry_msgs::Point>& points);

    // --- planner ---
  
    AStar _planner;

    std::mutex _controller_mutex;
    PurePursuitController _controller;
};