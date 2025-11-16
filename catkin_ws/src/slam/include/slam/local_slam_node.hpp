#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include <thread>
#include <mutex>

class LocalSlamNode
{
public:
    LocalSlamNode();
    ~LocalSlamNode();

    // ros callbacks
    void front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // getters / setters
    sensor_msgs::LaserScan get_laser_scan();
    nav_msgs::Odometry get_odom_filtered();

    // worker thread
    void slam_thread();

private:
    // ros node handle
    ros::NodeHandle _nh;

    // ros subscribers
    ros::Subscriber _front_scan_sub;
    ros::Subscriber _odom_filtered_sub;

    // ros publishers
    ros::Publisher _odom_slam_pub;
    ros::Publisher _map_slam_pub;

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
    nav_msgs::OccupancyGrid _map;               // dynamic map
};
