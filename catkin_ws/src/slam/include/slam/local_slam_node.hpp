#include "ros/ros.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "slam/DoubleOccupancyGrid.h"


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

    bool transform_point_cloud_to_odom(const sensor_msgs::PointCloud& cloud, sensor_msgs::PointCloud& cloud_transformed);

    // getters / setters
    sensor_msgs::LaserScan get_laser_scan();
    nav_msgs::Odometry get_odom_filtered();

    // odometry msg to tf2 transform
    tf2::Transform odom_msg_to_tf2(const nav_msgs::Odometry& odom_msg);

    // tf2 transform to odometry msg
    nav_msgs::Odometry tf2_to_odom_msg(const tf2::Transform& tf, const std::string& frame_id, const std::string& child_frame_id);

    // map utils
    void init_map();
    int world_to_map_index(double x, double y);
    void update_map(const sensor_msgs::LaserScan& scan, const nav_msgs::Odometry& pose);
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
    ros::Publisher _double_map_pub;


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
    slam::DoubleOccupancyGrid _double_map;      // double precision map for internal use

    bool _is_first_iteration = true;

    // map parameters
    const double MAP_RESOLUTION = 0.05; // meters per cell
    const int MAP_WIDTH = 2000;          // cells
    const int MAP_HEIGHT = 2000;         // cells




};
