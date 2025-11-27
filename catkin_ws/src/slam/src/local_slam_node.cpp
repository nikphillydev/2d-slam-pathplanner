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
    ros::Rate loop_rate(20);
    
    while(ros::ok())
    {
        ROS_INFO("slam thread running");
        ros::Time start;
        double duration;

        // Get the latest sensor data
        start = ros::Time::now();

        sensor_msgs::LaserScan laser_scan = get_laser_scan();
        nav_msgs::Odometry odom_filtered = get_odom_filtered();

        // Check both msgs have valid data
        if (laser_scan.header.stamp.toSec() == 0 || odom_filtered.header.stamp.toSec() == 0)
        {
            ROS_INFO("No data received, waiting...");
            loop_rate.sleep();
            continue;
        }

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("GET took: %f seconds", duration);

        // Extract point cloud from laser scan
        start = ros::Time::now();

        sensor_msgs::PointCloud cloud_laser = laser_scan_to_point_cloud(laser_scan, 0);
        sensor_msgs::PointCloud cloud_base;
        geometry_msgs::TransformStamped tf_base_to_laser;
        try{
            tf_base_to_laser = _tf_buffer.lookupTransform("base_link", cloud_laser.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            loop_rate.sleep();
            continue;
        }
        transform_point_cloud(cloud_laser, cloud_base, tf_base_to_laser);

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("PC LASER to BASE TRANSFORM took: %f seconds", duration);
        
        if (is_first_iteration)
        {
            _last_odom_filtered = odom_filtered;

            is_first_iteration = false;
            ROS_INFO("SLAM Starting!");
        }
        
        // --- Prediction Step using Odometry (from EKF) ---
        start = ros::Time::now();

        // Calculate differential motion since last cycle
        tf2::Transform tf_odom_to_base_prev = create_transform_from_odom(_last_odom_filtered);
        tf2::Transform tf_odom_to_base = create_transform_from_odom(odom_filtered);
        tf2::Transform tf_delta = tf_odom_to_base_prev.inverse() * tf_odom_to_base;

        // Calculate the last corrected SLAM pose
        tf2::Transform tf_map_to_base_prev = get_tf_map_to_odom() * tf_odom_to_base_prev;

        // Apply differential motion to previous correction
        tf2::Transform tf_map_to_base_guess = tf_map_to_base_prev * tf_delta;

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Prediction took: %f seconds", duration);

        // --- Correction Step using Ceres Solver ---
        start = ros::Time::now();

        tf2::Transform tf_map_to_base_corrected;
        {
            std::lock_guard<std::mutex> lock(_double_map_mutex);
            tf_map_to_base_corrected = run_ceres_solver(cloud_base, tf_map_to_base_guess, _double_map);
        }

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Correction took: %f seconds", duration);

        // --- Update Step ---
        start = ros::Time::now();

        set_tf_map_to_odom(tf_map_to_base_corrected * tf_odom_to_base.inverse());

        geometry_msgs::TransformStamped tf_map_to_base = create_transform_stamped_from_transform(tf_map_to_base_corrected, "map", "base_link");
        sensor_msgs::PointCloud cloud_map;
        transform_point_cloud(cloud_base, cloud_map, tf_map_to_base);

        geometry_msgs::Pose robot_pose_map;
        tf2::toMsg(tf_map_to_base_corrected, robot_pose_map);
        update_map(robot_pose_map, cloud_map);

        duration = (ros::Time::now() - start).toSec();
        ROS_INFO("Update took: %f seconds", duration);

        _last_odom_filtered = odom_filtered;

        loop_rate.sleep();
    }
}

void LocalSlamNode::publisher_thread()
{
    ros::Rate loop_rate(50);        // high rate TF publishing
    
    while(ros::ok())
    {
        // Broadcast dynamic TF: map -> odom
        geometry_msgs::TransformStamped tf_stamped = create_transform_stamped_from_transform(get_tf_map_to_odom(), "map", "odom");
        _tf_broadcaster.sendTransform(tf_stamped);
        
        // Publish map (throttled)
        static int map_pub_counter = 0;
        if (map_pub_counter++ > 50) 
        {
            _map = convert_double_map_to_occupancy_grid(get_double_map());
            _map_slam_pub.publish(_map); 
            map_pub_counter = 0;
        }

        loop_rate.sleep();
    }
}

// --- map utils ---

tf2::Transform LocalSlamNode::run_ceres_solver(
    const sensor_msgs::PointCloud& cloud_base, 
    const tf2::Transform& initial_guess, 
    const slam::DoubleOccupancyGrid& map)
{
    if (map.data.empty() || cloud_base.points.empty()) return initial_guess;

    // 1. Convert Initial Guess (TF2) to Optimization Array (double[3])
    double pose[3];
    pose[0] = initial_guess.getOrigin().x();
    pose[1] = initial_guess.getOrigin().y();
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(initial_guess.getRotation()).getRPY(roll, pitch, yaw);
    pose[2] = yaw;

    // save raw pose for xi_head
    double initial_x = pose[0];
    double initial_y = pose[1];
    double initial_theta = pose[2];

    // 2. Build Ceres Problem
    ceres::Problem problem;
    
    // We only use valid ranges
    // Step size can be increased (e.g., i+=2 or i+=4) for performance
    for (size_t i = 0; i < cloud_base.points.size(); i += 1) {
        double p_x = cloud_base.points[i].x;
        double p_y = cloud_base.points[i].y;

        // Create Cost Function
        // AutoDiffCostFunction handles the calculus (derivatives) automatically using templates.
        // <PointToMapResidual, 1, 3> means:
        //   1: Size of Residual (1 scalar error value)
        //   3: Size of Parameter (x, y, theta)
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<PointToMapResidual, 1, 3>(
                new PointToMapResidual(p_x, p_y,
                                     map.data, 
                                     map.info.width, map.info.height, 
                                     map.info.resolution,
                                     map.info.origin.position.x, 
                                     map.info.origin.position.y)
            );

        // Add Residual Block
        problem.AddResidualBlock(cost_function, nullptr, pose);
    }

    double ALPHA_POS = 1.0;
    double ALPHA_ROT = 5.0; // Higher weight on rotation to prevent large angular drift

    ceres::CostFunction* prior_cost_function =
        new ceres::AutoDiffCostFunction<PosePriorResidual, 3, 3>(
            new PosePriorResidual(initial_x, initial_y, initial_theta, 
                                  ALPHA_POS, ALPHA_ROT)
        );

    problem.AddResidualBlock(prior_cost_function, nullptr, pose);

    // 3. Configure Solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 30; // Keep it fast for real-time
    options.minimizer_progress_to_stdout = false;

    // 4. Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ROS_INFO("Ceres Solver: %s", summary.BriefReport().c_str());

    // 5. Convert Result back to TF2
    tf2::Transform result_tf;
    tf2::Vector3 origin(pose[0], pose[1], 0.0);
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, pose[2]); // Assuming 2D planar
    
    result_tf.setOrigin(origin);
    result_tf.setRotation(rotation);

    return result_tf;
}

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
    
    // origin represents the lower-left corner (not the geometric center) of the map in the map frame
    // set origin to try and place robot near geometric center
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
        // TODO update (double) the map size here
        ROS_WARN("WARNING: odom slam coordinates out of map bounds");
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
    
    // get robot index
    int r_idx = coord_to_map_index(r_x, r_y, _double_map.info);

    // Probability Constants
    const double P_HIT = 0.9;
    const double P_MISS = 0.1;

    for (const auto& point : cloud.points)
    {
        // Update HIT
        int hit_idx = coord_to_map_index(point.x, point.y, _double_map.info);
        if (hit_idx != -1)
        {
            // index within map bounds
            if (_double_map.data[hit_idx] == -1)
            {
                // unobserved grid point
                _double_map.data[hit_idx] = P_HIT;
            }
            else 
            {
                _double_map.data[hit_idx] = clamp(inv_odds(odds(_double_map.data[hit_idx]) * odds(P_HIT)));
            }
        }

        // Update MISS
        // Raytrace (Bresenham)

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
            // hit point reached
            if (x0 == x1 && y0 == y1) break;

            // bounds check current cell
            if (x0 >= 0 && x0 < _double_map.info.width && y0 >= 0 && y0 < _double_map.info.height) 
            {
                int miss_idx = y0 * _double_map.info.width + x0;
                if (_double_map.data[miss_idx] == -1)
                {
                    // unobserved grid point
                    _double_map.data[miss_idx] = P_MISS;
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

nav_msgs::OccupancyGrid LocalSlamNode::convert_double_map_to_occupancy_grid(const slam::DoubleOccupancyGrid& double_map)
{
    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header = double_map.header;
    occupancy_grid.info = double_map.info;
    occupancy_grid.data.resize(double_map.data.size());

    for (size_t i = 0; i < double_map.data.size(); ++i){
        if (double_map.data[i] == -1)
        {
            occupancy_grid.data[i] = -1; // Unknown
        }
        else
        {
            occupancy_grid.data[i] = static_cast<int8_t>(double_map.data[i] * 100); // Convert to [0, 100]
        }
    }
    return occupancy_grid;
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

slam::DoubleOccupancyGrid LocalSlamNode::get_double_map()
{
    std::lock_guard<std::mutex> lock(_double_map_mutex);
    return _double_map;
}