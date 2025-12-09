#pragma once

#include "ros/ros.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

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

    // --- threads ---
    
    void slam_thread();
    void publisher_thread();

    // --- slam ---
    
    tf2::Transform run_ceres_solver(
        const sensor_msgs::PointCloud& cloud_base, 
        const tf2::Transform& tf_map_to_base_guess);

    // --- map utility ---
    
    void init_map();

    int coord_to_map_index(double x, double y, const nav_msgs::MapMetaData& info);
    int coord_to_map_row(double x, const nav_msgs::MapMetaData& info);
    int coord_to_map_col(double y, const nav_msgs::MapMetaData& info);

    void update_map(const geometry_msgs::Pose& scan_origin, const sensor_msgs::PointCloud& cloud);
    
    void publish_map();
    
    // --- ros callbacks ---

    void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // --- thread safe getters / setters ---
    
    sensor_msgs::LaserScan get_laser_scan();
    void set_laser_scan(const sensor_msgs::LaserScan& scan);

    nav_msgs::Odometry get_odom_filtered();
    void set_odom_filtered(const nav_msgs::Odometry& odom);

    tf2::Transform get_tf_map_to_odom();
    void set_tf_map_to_odom(const tf2::Transform& tf);

private:
    // --- ros ---

    ros::NodeHandle _nh;

    ros::Subscriber _front_scan_sub;
    ros::Subscriber _odom_filtered_sub;

    ros::Publisher _map_slam_pub;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_broadcaster;

    // --- threads ---

    std::thread _slam_thread_handle;
    std::thread _publisher_thread_handle;

    // --- slam internal state ---

    // input
    std::mutex _input_mutex;;
    sensor_msgs::LaserScan _laser_scan;         // laser scan from LiDAR
    nav_msgs::Odometry _odom_filtered;          // odometry from ekf
    nav_msgs::Odometry _last_odom_filtered;

    // output
    std::mutex _tf_map_to_odom_mutex;
    tf2::Transform _tf_map_to_odom;             // robot odometry transform in map frame

    std::mutex _double_map_mutex;
    slam::DoubleOccupancyGrid _double_map;      // double precision probability map for internal use

    nav_msgs::OccupancyGrid _map;               // cached map for publishing

    // slam parameters
    const double MAP_RESOLUTION = 0.02;         // meters per cell
    const int MAP_WIDTH = 30;                   // meters
    const int MAP_HEIGHT = 30;                  // meters
    const int MAP_CELL_WIDTH = MAP_WIDTH / MAP_RESOLUTION;
    const int MAP_CELL_HEIGHT = MAP_HEIGHT / MAP_RESOLUTION;
    const double P_HIT = 0.95;                  // > 0.5. Higher P_HIT -> aggressive mapping: obstacles appear quicker, more sensitive to noise
    const double P_MISS = 0.45;                 // < 0.5. Lower P_MISS -> aggressive clearing: obstables remove quicker
    const double CERES_MAX_ITERATIONS = 50;
    const int CERES_POINT_STEP_SIZE = 2;        // increase for performance
    const int TF_PUBLISHER_RATE_HZ = 50;        // do not touch this
    const int SLAM_THREAD_LOOP_HZ = 3;          // do not touch this
};