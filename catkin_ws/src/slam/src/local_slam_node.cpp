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
    _odom_slam_pub = _nh.advertise<nav_msgs::Odometry>("/odometry/slam", 1000);
    _map_slam_pub = _nh.advertise<nav_msgs::OccupancyGrid>("/map/slam", 1000);
    _cloud_slam_pub = _nh.advertise<sensor_msgs::PointCloud>("/cloud/slam", 1000);

    // start worker thread
    _slam_thread_handle = std::thread(&LocalSlamNode::slam_thread, this);

    // Initialize
    init_map();
    broadcast_map_tf();

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
    bool is_first_iteration = true;
    
    ros::Rate loop_rate(1000);
    
    while(ros::ok())
    {
        ROS_INFO("slam thread running");

        sensor_msgs::LaserScan laser_scan = get_laser_scan();
        nav_msgs::Odometry odom_filtered = get_odom_filtered();

        // std::stringstream debug;
        // debug << "Laser scan frame: " << laser_scan.header.frame_id << " Odometry frame: " << odom_filtered.header.frame_id << std::endl;
        // ROS_INFO("%s", debug.str().c_str());

        // check both msgs have valid data
        if (laser_scan.header.stamp.toSec() == 0 || odom_filtered.header.stamp.toSec() == 0)
        {
            ROS_INFO("No data received, waiting...");
            loop_rate.sleep();
            continue;
        }

        // Extract point cloud from laser scan
        sensor_msgs::PointCloud cloud_laser = laser_scan_to_point_cloud(laser_scan, 0);
        sensor_msgs::PointCloud cloud_base_link;
        geometry_msgs::TransformStamped transform_laser_to_base_link;
        try{
            transform_laser_to_base_link = _tf_buffer.lookupTransform("base_link", cloud_laser.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            loop_rate.sleep();
            continue;
        }
        if (!transform_point_cloud(cloud_laser, cloud_base_link, transform_laser_to_base_link)) {
            loop_rate.sleep();
            continue;
        }


        if (is_first_iteration)
        {
            _odom_slam = odom_filtered;
            _odom_slam.child_frame_id = "base_link_slam";
            _odom_slam_tf = create_transform_from_odom(_odom_slam);
            is_first_iteration = false;
            ROS_INFO("SLAM Starting!");
        }
        else
        {   
            // A

            tf2::Transform tf_odom_prev = create_transform_from_odom(_last_odom_filtered);
            tf2::Transform tf_odom_curr = create_transform_from_odom(odom_filtered);
            tf2::Transform tf_delta = tf_odom_prev.inverse() * tf_odom_curr;

            // B
            // Odometry Prediction

            _odom_slam_tf = _odom_slam_tf * tf_delta;


            // C
            // Ceres Solver Update
            _odom_slam_tf = run_ceres_solver(cloud_base_link, _odom_slam_tf, _double_map);
            _odom_slam = create_odom_from_transform(_odom_slam_tf, "odom", "base_link_slam");
        }

        // D

        geometry_msgs::TransformStamped transform_base_link_slam_to_odom = create_transform_stamped_from_transform(_odom_slam_tf, "odom", "base_link_slam");

        sensor_msgs::PointCloud cloud_odom;
        if (!transform_point_cloud(cloud_base_link, cloud_odom, transform_base_link_slam_to_odom)) return;

        // E

        update_map(cloud_odom);

        // F

        _last_odom_filtered = odom_filtered;

        _odom_slam_pub.publish(_odom_slam);
        _map = convert_double_map_to_occupancy_grid();
        _map_slam_pub.publish(_map);
        _cloud_slam_pub.publish(cloud_odom);
        broadcast_map_tf();
        _tf_broadcaster.sendTransform(transform_base_link_slam_to_odom);

        loop_rate.sleep();
    }
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
    for (size_t i = 0; i < cloud_base.points.size(); i += 2) {
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

    double ALPHA_POS = 10.0;
    double ALPHA_ROT = 50.0; // Higher weight on rotation to prevent large angular drift

    ceres::CostFunction* prior_cost_function =
        new ceres::AutoDiffCostFunction<PosePriorResidual, 3, 3>(
            new PosePriorResidual(initial_x, initial_y, initial_theta, 
                                  ALPHA_POS, ALPHA_ROT)
        );

    problem.AddResidualBlock(prior_cost_function, nullptr, pose);

    // 3. Configure Solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20; // Keep it fast for real-time
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

void LocalSlamNode::init_map(){
    _double_map.header.frame_id = "map";
    _double_map.info.map_load_time = ros::Time::now();
    // Initialize occupancy grid map parameters
    _double_map.info.resolution = MAP_RESOLUTION;
    _double_map.info.width = MAP_WIDTH;
    _double_map.info.height = MAP_HEIGHT;
    _double_map.info.origin.position.x = 0; // Centered at (0,0)
    _double_map.info.origin.position.y = 0;
    _double_map.info.origin.position.z = 0.0;
    _double_map.info.origin.orientation.w = 1.0; // No rotation
    // Initialize map data to unknown (-1)
    _double_map.data.resize(_double_map.info.width * _double_map.info.height, -1);
}

void LocalSlamNode::broadcast_map_tf()
{
    geometry_msgs::Vector3 translation;
    translation.x = 0;
    translation.y = 0;
    translation.z = 0;
    geometry_msgs::Quaternion rotation;
    rotation.w = 1;
    rotation.x = 0;
    rotation.y = 0;
    rotation.z = 0;
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = "map";
    tf_stamped.child_frame_id = "odom";
    tf_stamped.transform.translation = translation;
    tf_stamped.transform.rotation = rotation;
    _tf_broadcaster.sendTransform(tf_stamped);
}

int LocalSlamNode::world_to_map_index(double x, double y)
{
    double map_x = (x - _double_map.info.origin.position.x) / _double_map.info.resolution;
    double map_y = (y - _double_map.info.origin.position.y) / _double_map.info.resolution;

    int i = (int)map_x;
    int j = (int)map_y;


    if (i < 0 || i >= _double_map.info.width || j < 0 || j >= _double_map.info.height)
    {
        // TODO update (double) the map size here
        ROS_WARN("World coordinates out of map bounds");
        return -1;
    }

    return j * _double_map.info.width + i;
}

// Bayesian Occupancy Grid Mapping Update
void LocalSlamNode::update_map(const sensor_msgs::PointCloud& cloud)
{
    // Probability Constants
    const double P_HIT = 0.55;
    const double P_MISS = 0.45;

    if (_double_map.data.empty()){
        init_map();
    }
    if (cloud.points.empty()){
        return;
    }

    // 1. Get current robot position (The origin of the rays)
    double robot_x = _odom_slam.pose.pose.position.x;
    double robot_y = _odom_slam.pose.pose.position.y;

    for (const auto& point : cloud.points){
        // 2. Update the "Hit" point
        int hit_index = world_to_map_index(point.x, point.y);
        if (hit_index != -1){
            if (_double_map.data[hit_index] == -1)
            {
                _double_map.data[hit_index] = P_HIT; // Init
            }
            _double_map.data[hit_index] = clamp(inv_odds(odds(_double_map.data[hit_index]) * odds(P_HIT)));
        }

        // 3. Ray Trace for "Miss" points (Free space between Robot and Wall)
        double dx = point.x - robot_x;
        double dy = point.y - robot_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Normalized direction vector
        double unit_x = dx / distance;
        double unit_y = dy / distance;

        // Step along the ray
        for (double r = 0; r < distance; r += MAP_RESOLUTION) {
            
            // Calculate intermediate point
            double check_x = robot_x + unit_x * r;
            double check_y = robot_y + unit_y * r;

            int miss_index = world_to_map_index(check_x, check_y);

            // Don't update if out of bounds or if we hit the wall itself
            if (miss_index == -1 || miss_index == hit_index) {
                break;
            }

            // Update free space
            if (_double_map.data[miss_index] == -1) 
            {
                _double_map.data[miss_index] = P_MISS; // Init
            }
            _double_map.data[miss_index] = clamp(inv_odds(odds(_double_map.data[miss_index]) * odds(P_MISS)));
        }
    }
}


nav_msgs::OccupancyGrid LocalSlamNode::convert_double_map_to_occupancy_grid(){
    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header = _double_map.header;
    occupancy_grid.info = _double_map.info;
    occupancy_grid.data.resize(_double_map.data.size());

    for (size_t i = 0; i < _double_map.data.size(); ++i){
        if (_double_map.data[i] == -1){
            occupancy_grid.data[i] = -1; // Unknown
        }
        else{
            occupancy_grid.data[i] = static_cast<int8_t>(_double_map.data[i] * 100); // Convert to [0, 100]
        }
    }
    return occupancy_grid;
}