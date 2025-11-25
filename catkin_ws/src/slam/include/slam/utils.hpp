#include <cmath>

#include "ros/ros.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

sensor_msgs::PointCloud laser_scan_to_point_cloud(const sensor_msgs::LaserScan& scan, float z_coord) 
{
    sensor_msgs::PointCloud cloud;
    cloud.header = scan.header;
    cloud.points.reserve(scan.ranges.size());
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        float range = scan.ranges[i];
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)
        {
            continue;
        }
        float angle = scan.angle_min + (i * scan.angle_increment);
        float x_coord = range * std::cos(angle);
        float y_coord = range * std::sin(angle);
        
        geometry_msgs::Point32 point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;

        cloud.points.push_back(point);
    }
    return cloud;
}

bool transform_point_cloud(const sensor_msgs::PointCloud& in_cloud, sensor_msgs::PointCloud& out_cloud, const geometry_msgs::TransformStamped& ts)
{
    out_cloud.header = in_cloud.header;
    out_cloud.header.frame_id = ts.header.frame_id;
    out_cloud.points.reserve(in_cloud.points.size());
    for (const geometry_msgs::Point32& point32 : in_cloud.points)
    {
        geometry_msgs::Point point_tf2_in;
        geometry_msgs::Point point_tf2_out;
        point_tf2_in.x = point32.x;
        point_tf2_in.y = point32.y;
        point_tf2_in.z = point32.z;
        tf2::doTransform(point_tf2_in, point_tf2_out, ts);

        geometry_msgs::Point32 point32_transformed;
        point32_transformed.x = static_cast<float>(point_tf2_out.x);
        point32_transformed.y = static_cast<float>(point_tf2_out.y);
        point32_transformed.z = 0;
        out_cloud.points.push_back(point32_transformed);
    }
    return true;
}

geometry_msgs::TransformStamped create_transform_stamped_from_odom(const nav_msgs::Odometry& odom)
{
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom.header.stamp;
    odom_trans.header.frame_id = odom.header.frame_id;          // odom
    odom_trans.child_frame_id = odom.child_frame_id;            // base_link
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
    return odom_trans;
}

tf2::Transform create_transform_from_odom(const nav_msgs::Odometry& odom)
{
    tf2::Transform tf;
    tf2::Vector3 translation(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z
    );
    tf2::Quaternion rotation(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    );
    tf.setOrigin(translation);
    tf.setRotation(rotation);
    return tf;
}

nav_msgs::Odometry create_odom_from_transform(const tf2::Transform& tf, const std::string& frame_id, const std::string& child_frame_id)
{
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;

    tf2::Vector3 translation = tf.getOrigin();
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();

    tf2::Quaternion rotation = tf.getRotation();
    odom_msg.pose.pose.orientation.x = rotation.x();
    odom_msg.pose.pose.orientation.y = rotation.y();
    odom_msg.pose.pose.orientation.z = rotation.z();
    odom_msg.pose.pose.orientation.w = rotation.w();

    return odom_msg;
}

geometry_msgs::TransformStamped create_transform_stamped_from_transform(const tf2::Transform& tf, const std::string& frame_id, const std::string& child_frame_id)
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform = tf2::toMsg(tf);
    return transform_stamped;
}

geometry_msgs::TransformStamped invert_transform(const geometry_msgs::TransformStamped& ts)
{
    tf2::Transform transform;
    tf2::fromMsg(ts.transform, transform);
    tf2::Transform transform_inv = transform.inverse();
    geometry_msgs::TransformStamped ts_inv;
    ts_inv.header = ts.header;
    ts_inv.child_frame_id = ts.header.frame_id;
    ts_inv.header.frame_id = ts.child_frame_id;
    ts_inv.transform = tf2::toMsg(transform_inv);
    return ts_inv;
}

double odds(double p){
    return p / (1.0 - p);
}
double inv_odds(double o){
    return (o) / (1.0 + o);
}
double clamp(double val)
{  
    // clamp value between 0.001 and 0.9
    if (val < 0.001) {
        return 0.001;
    } else if (val > 0.9){
        return 0.9;
    } else {
        return val;
    }
}