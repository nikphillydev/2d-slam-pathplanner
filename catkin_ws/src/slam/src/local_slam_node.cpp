#include "slam/local_slam_node.hpp"
#include "slam/utils.hpp"

LocalSlamNode::LocalSlamNode() 
    : _tf_listener(_tf_buffer)
{
    // create subscribers
    _front_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("/front/scan", 1000, &LocalSlamNode::front_scan_callback, this);
    _odom_filtered_sub = _nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, &LocalSlamNode::odom_filtered_callback, this);

    // create publishers
    _odom_slam_pub = _nh.advertise<nav_msgs::Odometry>("/odometry/slam", 1000);
    _map_slam_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/map/slam", 1000);
    _cloud_slam_pub = _nh.advertise<sensor_msgs::PointCloud>("/cloud/slam", 1000);

    // start worker thread
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

// --- worker thread ---

void LocalSlamNode::slam_thread()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ROS_INFO("slam thread running");

        sensor_msgs::LaserScan laser_scan = get_laser_scan();
        nav_msgs::Odometry odom_filtered = get_odom_filtered();

        // std::stringstream debug;
        // debug << "Laser scan frame: " << laser_scan.header.frame_id << " Odometry frame: " << odom_filtered.header.frame_id << std::endl;
        // ROS_INFO("%s", debug.str().c_str());
        
        _odom_slam = odom_filtered;

        sensor_msgs::PointCloud cloud_laser = laser_scan_to_point_cloud(laser_scan, 0);
        geometry_msgs::TransformStamped transform_laser_to_base_link;
        try{
            transform_laser_to_base_link = _tf_buffer.lookupTransform("base_link", cloud_laser.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        sensor_msgs::PointCloud cloud_base_link;
        if (!transform_point_cloud(cloud_laser, cloud_base_link, transform_laser_to_base_link)) return;

        geometry_msgs::TransformStamped transform_odom_to_base_link = create_transform_from_odom(_odom_slam);

        // calculate transform_base_link_to_odom
        // get cloud_odom

        // geometry_msgs::TransformStamped odom_transform = create_transform_from_odom(_odom_slam);
        // sensor_msgs::PointCloud cloud_odom;
        // transform_point_cloud_to_odom(cloud_laser_scane, cloud_odom);
        // _cloud_slam_pub.publish(cloud_odom);

        loop_rate.sleep();
    }
}

bool LocalSlamNode::transform_point_cloud_to_odom(const sensor_msgs::PointCloud& in_cloud, sensor_msgs::PointCloud& out_cloud)
{
    geometry_msgs::TransformStamped transform_stamped;
    try{
        transform_stamped = _tf_buffer.lookupTransform("odom", in_cloud.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
    out_cloud.header = in_cloud.header;
    out_cloud.header.frame_id = "odom";
    out_cloud.points.reserve(in_cloud.points.size());
    for (const geometry_msgs::Point32& point32 : in_cloud.points)
    {
        // ROS_INFO("Point before: %f, %f, %f", point32.x, point32.y, point32.z);
        geometry_msgs::Point point_tf2_in;
        point_tf2_in.x = point32.x;
        point_tf2_in.y = point32.y;
        point_tf2_in.z = point32.z;
        geometry_msgs::Point point_tf2_out;
        tf2::doTransform(point_tf2_in, point_tf2_out, transform_stamped);

        geometry_msgs::Point32 point32_transformed;
        point32_transformed.x = static_cast<float>(point_tf2_out.x);
        point32_transformed.y = static_cast<float>(point_tf2_out.y);
        point32_transformed.z = 0;
        ROS_INFO("Point after: %f, %f, %f", point32_transformed.x, point32_transformed.y, point32_transformed.z);
        out_cloud.points.push_back(point32_transformed);
    }
    return true;
}

// --- ros callbacks ---

void LocalSlamNode::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_laser_scan_mutex);
    _laser_scan = std::move(*msg);
}

void LocalSlamNode::odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(_odom_filtered_mutex);
    _odom_filtered = std::move(*msg);
}

// --- getters / setters ---

sensor_msgs::LaserScan LocalSlamNode::get_laser_scan()
{
    std::lock_guard<std::mutex> lock(_laser_scan_mutex);
    return _laser_scan;
}


nav_msgs::Odometry LocalSlamNode::get_odom_filtered()
{
    std::lock_guard<std::mutex> lock(_odom_filtered_mutex);
    return _odom_filtered;
}