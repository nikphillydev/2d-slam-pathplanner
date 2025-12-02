#include "slam/local_slam_node.hpp"
#include "slam/residual.hpp"
#include "slam/utils.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

LocalSlamNode::LocalSlamNode() 
    : _tf_listener(_tf_buffer)
{
    // create subscribers
    _front_scan_sub = _nh.subscribe<sensor_msgs::LaserScan>("/front/scan", 1000, &LocalSlamNode::front_scan_callback, this);
    _odom_filtered_sub = _nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, &LocalSlamNode::odom_filtered_callback, this);

    // create publishers
    _map_slam_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/map/slam", 1000);

    // initialization
    init_map();
    _tf_map_to_odom.setIdentity();      // align map frame with odom frame

    // start worker threads
    _slam_thread_handle = std::thread(&LocalSlamNode::slam_thread, this);
    _publisher_thread_handle = std::thread(&LocalSlamNode::publisher_thread, this);

    ROS_INFO("LocalSlamNode has started");
}

LocalSlamNode::~LocalSlamNode() 
{
    if (_slam_thread_handle.joinable()) _slam_thread_handle.join();
    if (_publisher_thread_handle.joinable()) _publisher_thread_handle.join();

    ROS_INFO("LocalSlamNode has stopped");
}

// --- worker threads ---

void LocalSlamNode::slam_thread()
{
    bool is_first_iteration = true;
    ros::Rate loop_rate(SLAM_THREAD_LOOP_HZ);
    
    while(ros::ok())
    {
        ROS_INFO("slam thread running");
        ros::Time start;
        double duration;

        // get the latest sensor data
        start = ros::Time::now();

        sensor_msgs::LaserScan laser_scan = get_laser_scan();
        nav_msgs::Odometry odom_filtered = get_odom_filtered();

        // check both msgs have valid data
        if (laser_scan.header.stamp.toSec() == 0 || odom_filtered.header.stamp.toSec() == 0)
        {
            ROS_INFO("No data received, waiting...");
            loop_rate.sleep();
            continue;
        }

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("GET took: %.6f seconds", duration);

        // extract point cloud from laser scan
        start = ros::Time::now();

        sensor_msgs::PointCloud cloud_laser = laser_scan_to_point_cloud(laser_scan, 0);
        sensor_msgs::PointCloud cloud_base;
        geometry_msgs::TransformStamped tfs_base_to_laser;
        try{
            tfs_base_to_laser = _tf_buffer.lookupTransform("base_link", cloud_laser.header.frame_id, cloud_laser.header.stamp);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            loop_rate.sleep();
            continue;
        }
        transform_point_cloud(cloud_laser, cloud_base, tfs_base_to_laser);

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("LASER to BASE_LINK TRANSFORM took: %.6f seconds", duration);
        
        if (is_first_iteration)
        {
            _last_odom_filtered = odom_filtered;

            is_first_iteration = false;
            ROS_INFO("SLAM Starting!");
        }
        
        // --- Prediction Step using Odometry (from EKF) ---
        start = ros::Time::now();

        // calculate differential motion since last cycle
        tf2::Transform tf_odom_to_base_prev = create_transform_from_odom(_last_odom_filtered);
        tf2::Transform tf_odom_to_base = create_transform_from_odom(odom_filtered);
        tf2::Transform tf_base_delta = tf_odom_to_base_prev.inverse() * tf_odom_to_base;

        // calculate the last corrected SLAM pose
        tf2::Transform tf_map_to_base_prev = get_tf_map_to_odom() * tf_odom_to_base_prev;

        // apply differential motion to last corrected SLAM pose
        tf2::Transform tf_map_to_base_guess = tf_map_to_base_prev * tf_base_delta;

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Prediction (Odometry) took: %.6f seconds", duration);

        // --- Correction Step using Ceres Solver ---
        start = ros::Time::now();

        // correct predicted SLAM pose using LiDAR scan
        tf2::Transform tf_map_to_base_corrected = run_ceres_solver(cloud_base, tf_map_to_base_guess);

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Correction (Ceres Solver) took: %.6f seconds", duration);

        // --- Update Step ---
        start = ros::Time::now();

        // odom -> base_link transformation handled by EKF
        // therefore we correct map -> odom transformation to counter accumulated drift
        set_tf_map_to_odom(tf_map_to_base_corrected * tf_odom_to_base.inverse());

        geometry_msgs::TransformStamped tfs_map_to_base_corrected = create_transform_stamped_from_transform(tf_map_to_base_corrected, "map", "base_link");
        sensor_msgs::PointCloud cloud_map;
        transform_point_cloud(cloud_base, cloud_map, tfs_map_to_base_corrected);

        // update grid probabilities
        geometry_msgs::Pose robot_pose_map;
        tf2::toMsg(tf_map_to_base_corrected, robot_pose_map);
        update_map(robot_pose_map, cloud_map);

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Update (Map) took: %.6f seconds", duration);

        _last_odom_filtered = odom_filtered;

        // reset laser scan
        sensor_msgs::LaserScan scan;
        set_laser_scan(scan);

        loop_rate.sleep();
    }
}

void LocalSlamNode::publisher_thread()
{
    ros::Rate loop_rate(TF_PUBLISHER_RATE_HZ);        // high rate TF publishing
    
    while(ros::ok())
    {
        // broadcast map -> odom transformation
        geometry_msgs::TransformStamped tf_stamped = create_transform_stamped_from_transform(get_tf_map_to_odom(), "map", "odom");
        _tf_broadcaster.sendTransform(tf_stamped);
        
        // publish map (throttled to once per second)
        static int map_pub_counter = 0;
        if (map_pub_counter++ > TF_PUBLISHER_RATE_HZ) 
        {
            publish_map();
            map_pub_counter = 0;
        }

        loop_rate.sleep();
    }
}

// --- slam ---

tf2::Transform LocalSlamNode::run_ceres_solver(
    const sensor_msgs::PointCloud& cloud_base, 
    const tf2::Transform& tf_map_to_base_guess)
{
    std::lock_guard<std::mutex> lock(_double_map_mutex);

    if (cloud_base.points.empty()) return tf_map_to_base_guess;

    // convert transform map -> base_link guess (TF2) into optimization array (double[3])
    double map_to_base[3];
    map_to_base[0] = tf_map_to_base_guess.getOrigin().x();
    map_to_base[1] = tf_map_to_base_guess.getOrigin().y();
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_map_to_base_guess.getRotation()).getRPY(roll, pitch, yaw);
    map_to_base[2] = yaw;

    // --- Build Ceres Problem ---
    ceres::Problem problem;
    
    for (size_t i = 0; i < cloud_base.points.size(); i += CERES_POINT_STEP_SIZE) 
    {
        // create cost function
        //   1: size of residual (scalar error)
        //   3: size of parameter (x, y, theta)
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<PointToMapResidual, 1, 3>(
                new PointToMapResidual(cloud_base.points[i], _double_map)
            );

        // add residual to Ceres
        // add loss function to prevents dynamic obstacles from dragging the map (removes outliers)
        // problem.AddResidualBlock(cost_function, nullptr, map_to_base);
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), map_to_base);
        // problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.15), map_to_base);
    }

    // configure and run solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = CERES_MAX_ITERATIONS;
    options.minimizer_progress_to_stdout = false;
    unsigned int num_threads = std::thread::hardware_concurrency();
    options.num_threads = (num_threads > 0) ? num_threads : 1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ROS_INFO_THROTTLE(1, "Ceres Solver: %s", summary.BriefReport().c_str());    // once per second

    // convert result back to TF2
    tf2::Transform map_to_base_optimized;
    tf2::Vector3 origin(map_to_base[0], map_to_base[1], 0.0);
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, map_to_base[2]);
    map_to_base_optimized.setOrigin(origin);
    map_to_base_optimized.setRotation(rotation);

    return map_to_base_optimized;
}

// --- map utility ---

void LocalSlamNode::init_map()
{
    std::lock_guard<std::mutex> lock(_double_map_mutex);

    _double_map.header.stamp = ros::Time::now();
    _double_map.header.frame_id = "map";

    // --- map metadata ---

    _double_map.info.map_load_time = ros::Time::now();       // the time at which the map was loaded
    _double_map.info.resolution = MAP_RESOLUTION;            // meters per cell
    _double_map.info.width = MAP_CELL_WIDTH;                 // cells
    _double_map.info.height = MAP_CELL_HEIGHT;               // cells
    
    // origin represents the lower-left corner (not the geometric center) of the map grid in the map frame
    // set origin to try and place robot near geometric center of the map grid
    _double_map.info.origin.position.x = -(_double_map.info.width * _double_map.info.resolution) / 2.0;
    _double_map.info.origin.position.y = -(_double_map.info.height * _double_map.info.resolution) / 2.0;
    _double_map.info.origin.orientation.w = 1.0;

    // --- initialize map ---

    // every cell unknown probability (-1)
    _double_map.data.resize(_double_map.info.width * _double_map.info.height, -1);
}

int LocalSlamNode::coord_to_map_index(double x, double y, const nav_msgs::MapMetaData& info)
{
    int map_x_index = coord_to_map_row(x, info);
    int map_y_index = coord_to_map_col(y, info);

    if (map_x_index < 0 || map_x_index >= info.width || 
        map_y_index < 0 || map_y_index >= info.height)
    {
        // TODO double the map size here OR implement sliding window (shift the map data and update the origin)
        ROS_WARN("WARNING: received coordinate out of map bounds");
        return -1;
    }

    return map_y_index * info.width + map_x_index;
}

int LocalSlamNode::coord_to_map_row(double x, const nav_msgs::MapMetaData& info)
{
    double map_x = x - info.origin.position.x;                                      // translate the coordinates to map origin
    int map_x_index = static_cast<int>(std::floor(map_x / info.resolution));        // scale by the resolution

    if (map_x_index < 0 || map_x_index >= info.width) return -1;

    return map_x_index;
}

int LocalSlamNode::coord_to_map_col(double y, const nav_msgs::MapMetaData& info)
{
    double map_y = y - info.origin.position.y;                                      // translate the coordinates to map origin
    int map_y_index = static_cast<int>(std::floor(map_y / info.resolution));        // scale by the resolution

    if (map_y_index < 0 || map_y_index >= info.height) return -1;

    return map_y_index;
}

void LocalSlamNode::update_map(const geometry_msgs::Pose& scan_origin, const sensor_msgs::PointCloud& cloud)
{
    std::lock_guard<std::mutex> lock(_double_map_mutex);

    _double_map.info.map_load_time = ros::Time::now();

    // robot position in map frame
    double r_x = scan_origin.position.x;
    double r_y = scan_origin.position.y;
    
    // get robot index in map
    int r_idx = coord_to_map_index(r_x, r_y, _double_map.info);

    for (const geometry_msgs::Point32& point : cloud.points)
    {
        // update HIT probabilities

        int hit_idx = coord_to_map_index(point.x, point.y, _double_map.info);
        if (hit_idx != -1)      // index within map bounds
        {
            if (_double_map.data[hit_idx] == -1)
            {
                _double_map.data[hit_idx] = P_HIT;      // previously unobserved grid point
            }
            else 
            {
                _double_map.data[hit_idx] = clamp(inv_odds(odds(_double_map.data[hit_idx]) * odds(P_HIT)));
            }
        }

        // update MISS probabilities, use Raytrace (Bresenham)

        // convert map coordinates to grid indexes
        int x0 = coord_to_map_row(r_x, _double_map.info);
        int y0 = coord_to_map_col(r_y, _double_map.info);
        int x1 = coord_to_map_row(point.x, _double_map.info);
        int y1 = coord_to_map_col(point.y, _double_map.info);

        // Bresenham initialization
        int dx = abs(x1 - x0);              // length of line in x
        int dy = abs(y1 - y0);              // length of line in y
        int sx = (x0 < x1) ? 1 : -1;        // step direction in x
        int sy = (y0 < y1) ? 1 : -1;        // step direction in y
        int err = dx - dy;                  // accumulated error

        while (true)
        {
            if (x0 == x1 && y0 == y1) break;        // HIT point reached

            // check current cell within map
            if (x0 >= 0 && x0 < _double_map.info.width && y0 >= 0 && y0 < _double_map.info.height) 
            {
                int miss_idx = y0 * _double_map.info.width + x0;
                if (_double_map.data[miss_idx] == -1)
                {
                    _double_map.data[miss_idx] = P_MISS;        // previously unobserved grid point
                }
                else 
                {
                    _double_map.data[miss_idx] = clamp(inv_odds(odds(_double_map.data[miss_idx]) * odds(P_MISS)));
                }
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

void LocalSlamNode::publish_map()
{
    std::lock_guard<std::mutex> lock(_double_map_mutex);
    _map.header = _double_map.header;
    _map.info = _double_map.info;
    _map.data.resize(_double_map.data.size());

    for (size_t i = 0; i < _double_map.data.size(); ++i){
        if (_double_map.data[i] == -1)
        {
            _map.data[i] = -1;        // unknown probability
        }
        else
        {
            // convert probability to [0, 100]
            _map.data[i] = static_cast<int8_t>(_double_map.data[i] * 100);
        }
    }
    _map_slam_pub.publish(_map);
}

// --- ros callbacks ---

void LocalSlamNode::front_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    set_laser_scan(*msg);
}

void LocalSlamNode::odom_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    set_odom_filtered(*msg);
}

// --- getters / setters ---

sensor_msgs::LaserScan LocalSlamNode::get_laser_scan()
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    return _laser_scan;
}

void LocalSlamNode::set_laser_scan(const sensor_msgs::LaserScan& scan)
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    _laser_scan = scan;
}

nav_msgs::Odometry LocalSlamNode::get_odom_filtered()
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    return _odom_filtered;
}

void LocalSlamNode::set_odom_filtered(const nav_msgs::Odometry& odom)
{
    std::lock_guard<std::mutex> lock(_input_mutex);
    _odom_filtered = odom;
}

tf2::Transform LocalSlamNode::get_tf_map_to_odom()
{
    std::lock_guard<std::mutex> lock(_tf_map_to_odom_mutex);
    return _tf_map_to_odom;
}

void LocalSlamNode::set_tf_map_to_odom(const tf2::Transform& tf)
{
    std::lock_guard<std::mutex> lock(_tf_map_to_odom_mutex);
    _tf_map_to_odom = tf;
}