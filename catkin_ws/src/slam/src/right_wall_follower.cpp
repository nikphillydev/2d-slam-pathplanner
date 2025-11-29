#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <algorithm>
#include <limits>
#include <cmath>

class RightWallFollower
{
public:
    RightWallFollower()
    : nh_("~")
    {
        nh_.param("scan_topic", scan_topic_, std::string("/front/scan"));
        nh_.param("cmd_topic",  cmd_topic_,  std::string("/cmd_vel"));

        nh_.param("desired_distance_from_wall", desired_distance_, 0.4);
        nh_.param("kp_wall",                kp_wall_,               1.5);
        nh_.param("base_linear_speed",      base_linear_speed_,     0.4);
        nh_.param("max_angular_speed",      max_angular_speed_,     1.0);

        nh_.param("front_obstacle_distance",       front_obstacle_dist_,       0.6);
        nh_.param("right_front_obstacle_distance", right_front_obstacle_dist_, 0.55);
        nh_.param("emergency_stop_distance",       emergency_stop_dist_,       0.30);

        nh_.param("right_sector_start", right_sector_start_, -1.92); // -110°
        nh_.param("right_sector_end",   right_sector_end_,   -1.22); // -70°

        nh_.param("right_front_sector_start", right_front_sector_start_, -1.22); // -70°
        nh_.param("right_front_sector_end",   right_front_sector_end_,   -0.35); // -20°

        nh_.param("front_sector_start", front_sector_start_, -0.35); // -20°
        nh_.param("front_sector_end",   front_sector_end_,    0.35); //  20°

        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
            scan_topic_, 1, &RightWallFollower::scanCallback, this);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

        ROS_INFO_STREAM("RightWallFollower started. scan_topic=" << scan_topic_
                        << " cmd_topic=" << cmd_topic_
                        << " desired_distance_from_wall=" << desired_distance_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher  cmd_pub_;

    std::string scan_topic_;
    std::string cmd_topic_;

    double desired_distance_;
    double kp_wall_;
    double base_linear_speed_;
    double max_angular_speed_;

    double front_obstacle_dist_;
    double right_front_obstacle_dist_;
    double emergency_stop_dist_;

    double right_sector_start_;
    double right_sector_end_;
    double right_front_sector_start_;
    double right_front_sector_end_;
    double front_sector_start_;
    double front_sector_end_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        const sensor_msgs::LaserScan& scan = *msg;

        double right_dist       = sectorMinRange(scan, right_sector_start_,       right_sector_end_);
        double right_front_dist = sectorMinRange(scan, right_front_sector_start_, right_front_sector_end_);
        double front_dist       = sectorMinRange(scan, front_sector_start_,       front_sector_end_);

        geometry_msgs::Twist cmd;
        cmd.linear.x  = base_linear_speed_;
        cmd.angular.z = 0.0;

        bool front_blocked       = std::isfinite(front_dist)       && (front_dist       < front_obstacle_dist_);
        bool right_front_blocked = std::isfinite(right_front_dist) && (right_front_dist < right_front_obstacle_dist_);

        if (front_blocked || right_front_blocked)
        {
            bool emergency = (std::isfinite(front_dist)       && front_dist       < emergency_stop_dist_) ||
                             (std::isfinite(right_front_dist) && right_front_dist < emergency_stop_dist_);

            if (emergency) {
                cmd.linear.x  = -0.05;
                cmd.angular.z = 0.8;
                ROS_WARN_THROTTLE(0.5, "EMERGENCY corner avoidance: front=%.2f, right_front=%.2f",
                                  front_dist, right_front_dist);
            } else {
                cmd.linear.x  = 0.0;
                cmd.angular.z = 0.6;
                ROS_INFO_THROTTLE(0.5, "Corner avoidance: front=%.2f, right_front=%.2f",
                                  front_dist, right_front_dist);
            }

            cmd_pub_.publish(cmd);
            return;
        }

        if (std::isfinite(right_dist)) {
            double error = desired_distance_ - right_dist;
            double angular_z = -kp_wall_ * error;

            if (angular_z >  max_angular_speed_) angular_z =  max_angular_speed_;
            if (angular_z < -max_angular_speed_) angular_z = -max_angular_speed_;

            cmd.angular.z = angular_z;

            if (right_dist < desired_distance_) {
                double scale = std::max(0.3, right_dist / desired_distance_);
                cmd.linear.x = base_linear_speed_ * scale;
            }

            ROS_DEBUG_STREAM_THROTTLE(0.5,
                "Follow wall: right=" << right_dist
                << " front=" << front_dist
                << " cmd.linear.x=" << cmd.linear.x
                << " cmd.angular.z=" << cmd.angular.z);
        } else {
            cmd.linear.x  = 0.2;
            cmd.angular.z = -0.4;
            ROS_INFO_THROTTLE(0.5, "Searching for right wall...");
        }

        cmd_pub_.publish(cmd);
    }
    double sectorMinRange(const sensor_msgs::LaserScan& scan,
                          double angle_start, double angle_end)
    {
        if (angle_start > angle_end) std::swap(angle_start, angle_end);

        int start_idx = static_cast<int>(std::ceil((angle_start - scan.angle_min) / scan.angle_increment));
        int end_idx   = static_cast<int>(std::floor((angle_end   - scan.angle_min) / scan.angle_increment));

        start_idx = std::max(0, start_idx);
        end_idx   = std::min(static_cast<int>(scan.ranges.size()) - 1, end_idx);

        if (end_idx < start_idx) {
            return std::numeric_limits<double>::infinity();
        }

        double min_r = std::numeric_limits<double>::infinity();
        for (int i = start_idx; i <= end_idx; ++i) {
            double r = scan.ranges[i];
            if (!std::isfinite(r))                continue;
            if (r < scan.range_min || r > scan.range_max) continue;
            if (r < min_r) min_r = r;
        }
        return min_r;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "right_wall_follower");
    RightWallFollower node;
    ros::spin();
    return 0;
}
