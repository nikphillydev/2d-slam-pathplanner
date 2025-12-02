#include "planning/planning_node.hpp"

PathPlanningNode::PathPlanningNode()
    : _tf_buffer(), _tf_listener(_tf_buffer)
{
    // create subscribers
    _map_slam_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("/map/slam", 1000, &PathPlanningNode::map_slam_callback, this);
    _goal_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/goal", 1, &PathPlanningNode::goal_callback, this);
    _laser_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("/front/scan", 1, &PathPlanningNode::laser_scan_callback, this);

    // create publishers
    _path_pub = _nh.advertise<nav_msgs::Path>("/path/planning", 1);
    _estop_pub = _nh.advertise<std_msgs::Bool>("/e_stop", 1);
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // start worker threads
    _planning_thread_handle = std::thread(&PathPlanningNode::planning_thread, this);
    // _emergency_stop_thread_handle = std::thread(&PathPlanningNode::emergency_stop_thread, this);
    _controller_thread_handle = std::thread(&PathPlanningNode::controller_thread, this);

    ROS_INFO("PathPlanningNode has started. Waiting for map and goal...");
}

PathPlanningNode::~PathPlanningNode()
{
    if (_planning_thread_handle.joinable())
        _planning_thread_handle.join();
    if (_controller_thread_handle.joinable())
        _controller_thread_handle.join();
    if (_emergency_stop_thread_handle.joinable())
        _emergency_stop_thread_handle.join();

    ROS_INFO("PathPlanningNode has stopped");
}

// --- worker threads ---

void PathPlanningNode::planning_thread()
{
    ros::Rate loop_rate(5);
    
    while(ros::ok())
    {
        ROS_INFO("path planning running");
      
        nav_msgs::OccupancyGrid map = get_map();
        if (map.data.empty())
        {
            ROS_WARN_THROTTLE(2, "No map received yet, waiting...");
            loop_rate.sleep();
            continue;
        }
      
        if (!_is_navigating.load())
        {
            ROS_WARN_THROTTLE(2, "No goal received yet, waiting...");
            loop_rate.sleep();
            continue;
        }
      
        geometry_msgs::Point start_point;
        geometry_msgs::TransformStamped tf_map_to_base_link;
        try {
            tf_map_to_base_link = _tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
            start_point.x = tf_map_to_base_link.transform.translation.x;
            start_point.y = tf_map_to_base_link.transform.translation.y;
            start_point.z = 0.0;
            ROS_INFO("Start Point: (%.2f, %.2f)", start_point.x, start_point.y);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            loop_rate.sleep();
            continue;
        }
        
        ros::Time start = ros::Time::now();
      
        geometry_msgs::Point goal_point = get_goal();
        std::vector<geometry_msgs::Point> path_points;   
        bool success = _planner.make_plan(start_point, goal_point, map, path_points);
      
        double duration = (ros::Time::now() - start).toSec();
        
        try {
            tf_map_to_base_link = _tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            loop_rate.sleep();
            continue;
        }

        if (success) 
        {
            ROS_INFO("A* path found with %lu points in %f seconds", path_points.size(), duration);
            {
                std::lock_guard<std::mutex> lock(_controller_mutex);
                _controller.initialize_path(path_points, tf_map_to_base_link.transform);
            }
            publish_path(path_points);
        } 
        else 
        {
            ROS_INFO("A* Failed to find a path!");
        }

        loop_rate.sleep();
    }
}

void PathPlanningNode::emergency_stop_thread()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sensor_msgs::LaserScan scan = get_laser_scan();
        float mindis = *std::min_element(scan.ranges.begin(), scan.ranges.end());
        if (mindis < EMERGENCY_STOP_DISTANCE)
        {
            std_msgs::Bool estop_msg;
            estop_msg.data = true;
            _estop_pub.publish(estop_msg);
        }
        else
        {
            std_msgs::Bool estop_msg;
            estop_msg.data = false;
            _estop_pub.publish(estop_msg);
        }
    }
}
  
void PathPlanningNode::controller_thread()
{
    ros::Rate loop_rate(50);
    
    while(ros::ok())
    {
        if (!_is_navigating.load())
        {
            ROS_WARN_THROTTLE(2, "No goal received yet, waiting...");
            loop_rate.sleep();
            continue;
        }

        geometry_msgs::TransformStamped tf_map_to_base_link;
        try {
            tf_map_to_base_link = _tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            loop_rate.sleep();
            continue;
        }

        geometry_msgs::Twist cmd_vel;
        bool path_following_complete = false;
        {
            std::lock_guard<std::mutex> lock(_controller_mutex);
            path_following_complete = _controller.follow_path(tf_map_to_base_link.transform, cmd_vel);
        }

        // set flag to stop navigation
        if (path_following_complete) _is_navigating.store(false);

        _cmd_vel_pub.publish(cmd_vel);
        loop_rate.sleep();
    }
}

// --- ros callbacks ---

void PathPlanningNode::map_slam_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    set_map(*msg);
}

void PathPlanningNode::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("PathPlanningNode received goal point!");
    if (msg->header.frame_id == "map")
    {
        set_goal(msg->pose.position);
        _is_navigating.store(true);
    }
}

void PathPlanningNode::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
    set_laser_scan(*msg);
}

// --- helpers ---
  
void PathPlanningNode::publish_path(const std::vector<geometry_msgs::Point>& points)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    for (const auto& point : points) 
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.0;
        
        path_msg.poses.push_back(pose);
    }

    _path_pub.publish(path_msg);
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
    for (auto& cell : _map.data) 
    {
        if (cell == -1) 
        {
            cell = 0;
        }
    }
}

geometry_msgs::Point PathPlanningNode::get_goal()
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    return _goal;
}

void PathPlanningNode::set_goal(const geometry_msgs::Point& goal)
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    _goal = goal;
}

sensor_msgs::LaserScan PathPlanningNode::get_laser_scan()
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    return _laser_scan;
}

void PathPlanningNode::set_laser_scan(const sensor_msgs::LaserScan& scan)
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    _laser_scan = scan;
}