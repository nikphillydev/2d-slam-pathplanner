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

        // check both msgs are not blank
        if (laser_scan.header.stamp.toSec() == 0 || odom_filtered.header.stamp.toSec() == 0){
            loop_rate.sleep();
            continue;
        }

        // first iteration -> odom_slam = odom_filtered

        if (_is_first_iteration){
            _odom_slam = odom_filtered;
            _last_odom_filtered = odom_filtered;
            _is_first_iteration = false;
            ROS_INFO("SLAM initialized at first frame");
        }
        else{   // Prediction
            // A
            tf2::Transform tf_odom_prev = odom_msg_to_tf2(_last_odom_filtered);
            tf2::Transform tf_odom_curr = odom_msg_to_tf2(odom_filtered);
            tf2::Transform tf_delta = tf_odom_prev.inverse() * tf_odom_curr;

            // B
            tf2::Transform tf_slam_prev = odom_msg_to_tf2(_odom_slam);
            tf2::Transform tf_slam_pred = tf_slam_prev * tf_delta;

            // C Ceres Solver Solving
            // tf_slam_corrected = run_ceres_solver(laser_scan, tf_slam_pred, map);

            tf2::Transform tf_slam_corrected = tf_slam_pred; // placeholder for corrected pose

            _odom_slam = tf2_to_odom_msg(tf_slam_corrected, "odom", "base_link");
            

            // D
            // update map
            
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

        }

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

// --- tf2 transformations ---

tf2::Transform LocalSlamNode::odom_msg_to_tf2(const nav_msgs::Odometry& odom_msg)
{
    tf2::Transform tf;
    
    tf2::Vector3 translation(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z
    );

    tf2::Quaternion rotation(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    );

    tf.setOrigin(translation);
    tf.setRotation(rotation);

    return tf;
}

nav_msgs::Odometry LocalSlamNode::tf2_to_odom_msg(const tf2::Transform& tf, const std::string& frame_id, const std::string& child_frame_id)
{
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time::now();  //timestamp should be the same as scan data
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;

    odom_msg.pose.pose.position.x = tf.getOrigin().x();
    odom_msg.pose.pose.position.y = tf.getOrigin().y();
    odom_msg.pose.pose.position.z = tf.getOrigin().z();

    tf2::Quaternion rotation = tf.getRotation();
    odom_msg.pose.pose.orientation.x = rotation.x();
    odom_msg.pose.pose.orientation.y = rotation.y();
    odom_msg.pose.pose.orientation.z = rotation.z();
    odom_msg.pose.pose.orientation.w = rotation.w();

    return odom_msg;
}


// --- map utils ---
void LocalSlamNode::init_map(){
    _double_map.header.frame_id = "map";
    _double_map.info.map_load_time = ros::Time::now();
    // Initialize occupancy grid map parameters
    _double_map.info.resolution = MAP_RESOLUTION;
    _double_map.info.width = MAP_WIDTH;
    _double_map.info.height = MAP_HEIGHT;
    _double_map.info.origin.position.x = - (MAP_WIDTH * MAP_RESOLUTION) / 2.0; // Centered at (0,0)
    _double_map.info.origin.position.y = - (MAP_HEIGHT * MAP_RESOLUTION) / 2.0;
    _double_map.info.origin.position.z = 0.0;
    _double_map.info.origin.orientation.w = 1.0; // No rotation
    // Initialize map data to unknown (-1)
    _double_map.data.resize(_double_map.info.width * _double_map.info.height, -1);
}

int LocalSlamNode::world_to_map_index(double x, double y){
    double map_x = (x - _double_map.info.origin.position.x) / _double_map.info.resolution;
    double map_y = (y - _double_map.info.origin.position.y) / _double_map.info.resolution;

    int i = (int)map_x;
    int j = (int)map_y;

    if (i < 0 || i >= _double_map.info.width || j < 0 || j >= _double_map.info.height){
        ROS_WARN("World coordinates out of map bounds");    // Do we update (double) the map size here?
        return -1;
    }

    return j * _double_map.info.width + i;
}

// Bayesian Occupancy Grid Mapping Update
void LocalSlamNode::update_map(const sensor_msgs::PointCloud& cloud){
    double P_HIT = 1 / exp(1);   // Probability of hit

    if (_double_map.data.empty()){
        init_map();
    }
    if (cloud.points.empty()){
        return;
    }
    // For each point in the point cloud
    for (const auto& point : cloud.points){
        int index = world_to_map_index(point.x, point.y);
        if (index == -1){
            continue;   // Skip points out of bounds
        }
        // Update occupancy probability using log-odds
        if (_double_map.data[index] == -1){
            _double_map.data[index] = P_HIT; // Initialize hit point to be 2/3 probability
        }
        else{
            //Mnew(x) = clamp(odds−1(odds(Mold(x)) · odds(phit)))
            _double_map.data[index] = clamp(inv_odds(odds(_double_map.data[index]) * odds(P_HIT)));
        }
        // for those grids along the ray, we update them as miss points
        for (i = 0; i < point.x; i += MAP_RESOLUTION){
            int miss_index = world_to_map_index(i, point.y/point.x * i);
            if (miss_index == -1 || miss_index == index){
                continue;
            }
            if (_double_map.data[miss_index] == -1){
                _double_map.data[miss_index] = 1 / (exp(1)*(point.x*point.x))*i*i; // Initialize miss point to be a x squared probability where the max is 1/e
            }
            else{
                // Mnew(x) = clamp(odds−1(odds(Mold(x)) · odds(pmiss)))
                double P_MISS = 1 / (exp(1)*(point.x*point.x))*i*i;
                _double_map.data[miss_index] = clamp(inv_odds(odds(_double_map.data[miss_index]) * odds(P_MISS)));
            }
        }
    }
}

nav_msgs::OccupancyGrid convert_double_map_to_occupancy_grid(){
    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header = _double_map.header;
    occupancy_grid.info = _double_map.info;
    occupancy_grid.data.resize(_double_map.data.size());

    for (size_t i = 0; i < _double_map.data.size(); ++i){
        if (_double_map.data[i] == -1){
            occupancy_grid.data[i] = -1; // Unknown
        }
        else{
            occupancy_grid.data[i] = static_cast<int8_t>(clamp(_double_map.data[i] * 100)); // Convert to [0, 100]
        }
    }
    return occupancy_grid;
}