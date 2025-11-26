#pragma once

#include "ros/ros.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"

#include "slam/DoubleOccupancyGrid.h"

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <cmath>

#include <thread>
#include <mutex>

class LocalSlamNode
{
public:
    LocalSlamNode();
    ~LocalSlamNode();

    // worker thread
    void slam_thread();

    // ros callbacks
    void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // getters / setters
    sensor_msgs::LaserScan get_laser_scan();
    nav_msgs::Odometry get_odom_filtered();

    // map utils
    tf2::Transform run_ceres_solver(
        const sensor_msgs::PointCloud& cloud_base, 
        const tf2::Transform& initial_guess, 
        const slam::DoubleOccupancyGrid& map);
    void init_map();
    void broadcast_map_tf();
    int world_to_map_index(double x, double y);
    void update_map(const sensor_msgs::PointCloud& cloud);
    nav_msgs::OccupancyGrid convert_double_map_to_occupancy_grid();
    

private:
    // ros node handle
    ros::NodeHandle _nh;

    // ros subscribers
    ros::Subscriber _front_scan_sub;
    ros::Subscriber _odom_filtered_sub;

    // ros publishers
    ros::Publisher _odom_slam_pub;
    ros::Publisher _map_slam_pub;
    ros::Publisher _cloud_slam_pub;

    // ros tf2
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_broadcaster;

    // worker thread
    std::thread _slam_thread_handle;

    // --- slam internal state ---

    // slam algorithm input
    std::mutex _laser_scan_mutex;
    sensor_msgs::LaserScan _laser_scan;         // laser scan from LiDAR
    std::mutex _odom_filtered_mutex;
    nav_msgs::Odometry _odom_filtered;          // odometry from ekf
    nav_msgs::Odometry _last_odom_filtered;

    // slam algorithm output
    nav_msgs::Odometry _odom_slam;              // odometry
    tf2::Transform _odom_slam_tf;               // odometry transform
    nav_msgs::OccupancyGrid _map;               // dynamic map
    slam::DoubleOccupancyGrid _double_map;      // double precision map for internal use

    // map parameters
    const double MAP_RESOLUTION = 0.005; // meters per cell
    const int MAP_WIDTH = 10000;          // cells
    const int MAP_HEIGHT = 10000;         // cells




};



