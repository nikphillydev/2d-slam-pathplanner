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
    void init_map();
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
    const double MAP_RESOLUTION = 0.05; // meters per cell
    const int MAP_WIDTH = 2000;          // cells
    const int MAP_HEIGHT = 2000;         // cells




};





inline double ExtractScalar(double value) {
  return value;
}

template <typename S, int N>
double ExtractScalar(const ceres::Jet<S, N>& value) {
  return static_cast<double>(value.a);
}

// Msmooth (bicubic interpolation)
double cubicInterpolate(double p0, double p1, double p2, double p3, double x) {
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 +
      x * (3.0 * (p1 - p2) + p3 - p0)));
}
double getGridValue(const slam::DoubleOccupancyGrid& grid, int x, int y) {
  int width = grid.info.width;
  int height = grid.info.height;
  if (x < 0) x = 0;
  if (x >= width) x = width - 1;
  if (y < 0) y = 0;
  if (y >= height) y = height - 1;
  return grid.data[y * width + x];
}

double Msmooth(const slam::DoubleOccupancyGrid& grid, double x, double y) {
    int ix = static_cast<int>(std::floor(x));
    int iy = static_cast<int>(std::floor(y));
    double fx = x - ix;
    double fy = y - iy;

    // cubic interpolation in x direction
    double col[4];
    for (int j = -1; j <= 2; ++j) {
        double p0 = getGridValue(grid, ix-1, iy+j);
        double p1 = getGridValue(grid, ix, iy+j);
        double p2 = getGridValue(grid, ix+1, iy+j);
        double p3 = getGridValue(grid, ix+2, iy+j);
        col[j+1] = cubicInterpolate(p0, p1, p2, p3, fx);
    }

    // cubic interpolation in y direction
    double value = cubicInterpolate(col[0], col[1], col[2], col[3], fy);
    return value;
}

struct ScanMatchCostFunctor{
    geometry_msgs::Point32 hk_;
    const slam::DoubleOccupancyGrid *grid_;

  ScanMatchCostFunctor(const geometry_msgs::Point32& hk,
             const slam::DoubleOccupancyGrid *grid)
    : hk_(hk), grid_(grid) {}
    
    template <typename T>
    bool operator()(const T* const xi, T* residual) const {
        const T& x = xi[0];
        const T& y = xi[1];
        const T& theta = xi[2];
        // coordinate transformation
        T cos_t = ceres::cos(theta);
        T sin_t = ceres::sin(theta);
        T hx = T(hk_.x);
        T hy = T(hk_.y);
        T wx = x + cos_t * hx - sin_t * hy;
        T wy = y + sin_t * hx + cos_t * hy;
        
        double wx_d = ExtractScalar(wx);
        double wy_d = ExtractScalar(wy);
        double m_d = Msmooth(*grid_, wx_d, wy_d);
        T m = T(m_d);
        residual[0] = T(1.0) - m;
        return true;
    }
};
