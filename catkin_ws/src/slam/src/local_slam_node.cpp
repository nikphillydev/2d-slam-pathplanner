#include "slam/local_slam_node.hpp"

LocalSlamNode::LocalSlamNode() 
{
    // subscribers
    _front_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("/front/scan", 1000, &LocalSlamNode::front_scan_callback, this);
    _odom_filtered_sub = _nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, &LocalSlamNode::odom_filtered_callback, this);

    // publishers
    _odom_slam_pub = _nh.advertise<nav_msgs::Odometry>("/odometry/slam", 1000);
    _map_slam_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/map/slam", 1000);

    // thread
    _slam_thread_handle = std::thread(&LocalSlamNode::slam_thread, this);

    ROS_INFO("LocalSlamNode has started");
}

LocalSlamNode::~LocalSlamNode() 
{
    if (_slam_thread_handle.joinable())
    {
        _slam_thread_handle.join();
    }

    ROS_INFO("LocalSlamNode has stopped");
}

void LocalSlamNode::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_laser_scan_mutex);
    _current_laser_scan = std::move(*msg);
}

void LocalSlamNode::odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_odom_filtered_mutex);
    _current_odom_filtered = std::move(*msg);
}

sensor_msgs::LaserScan LocalSlamNode::get_current_laser_scan()
{
    std::lock_guard<std::mutex> lock(_laser_scan_mutex);
    return _current_laser_scan;
}


nav_msgs::Odometry LocalSlamNode::get_current_odom_filtered()
{
    std::lock_guard<std::mutex> lock(_odom_filtered_mutex);
    return _current_odom_filtered;
}

void LocalSlamNode::slam_thread()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ROS_INFO("slam thread running");

        sensor_msgs::LaserScan new_scan = get_current_laser_scan();
        nav_msgs::Odometry new_odom = get_current_odom_filtered();

        std::stringstream debug;
        debug << "Laser scan: " << new_scan.header.stamp << " Odometry: " << new_odom.header.stamp << std::endl;
        ROS_INFO("%s", debug.str().c_str());

        loop_rate.sleep();
    }
}