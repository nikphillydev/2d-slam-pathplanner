#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class SimpleNavNode {
public:
    SimpleNavNode() {
        _goal_sub = _nh.subscribe("goal", 1, &SimpleNavNode::goal_callback, this);
        
        _odom_sub = _nh.subscribe("/odometry/slam", 1, &SimpleNavNode::odom_callback, this);
        
        _cmd_pub = _nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 10);

        ROS_INFO("Simple Navigation Node Started. Waiting for goal...");
    }

    void run() {
        ros::Rate rate(20);
        while (ros::ok()) {
            if (_has_goal) {
                control_loop();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;
    ros::Publisher _cmd_pub;

    geometry_msgs::PoseStamped _current_goal;
    nav_msgs::Odometry _current_odom;
    bool _has_goal = false;

    const double DIST_TOLERANCE = 0.1;
    const double ANGLE_TOLERANCE = 0.05;
    const double MAX_LINEAR_SPEED = 0.5;
    const double MAX_ANGULAR_SPEED = 1.0;

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        _current_goal = *msg;
        _has_goal = true;
        ROS_INFO("New Goal Received: x=%f, y=%f", _current_goal.pose.position.x, _current_goal.pose.position.y);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        _current_odom = *msg;
    }

    void control_loop() {
        double cx = _current_odom.pose.pose.position.x;
        double cy = _current_odom.pose.pose.position.y;
        
        tf2::Quaternion q(
            _current_odom.pose.pose.orientation.x,
            _current_odom.pose.pose.orientation.y,
            _current_odom.pose.pose.orientation.z,
            _current_odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        double gx = _current_goal.pose.position.x;
        double gy = _current_goal.pose.position.y;

        double dx = gx - cx;
        double dy = gy - cy;
        double distance = std::sqrt(dx*dx + dy*dy);

        double target_yaw = std::atan2(dy, dx);
        double angle_error = target_yaw - current_yaw;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        geometry_msgs::Twist cmd;

        if (distance < DIST_TOLERANCE) {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            _has_goal = false;
            ROS_INFO("Goal Reached!");
        }

        else if (std::abs(angle_error) > ANGLE_TOLERANCE) {
            cmd.linear.x = 0;
            cmd.angular.z = 2.0 * angle_error; 
            

            if (cmd.angular.z > MAX_ANGULAR_SPEED) cmd.angular.z = MAX_ANGULAR_SPEED;
            if (cmd.angular.z < -MAX_ANGULAR_SPEED) cmd.angular.z = -MAX_ANGULAR_SPEED;
        }

        else {
            cmd.linear.x = 0.5 * distance;
            cmd.angular.z = 1.0 * angle_error;

            if (cmd.linear.x > MAX_LINEAR_SPEED) cmd.linear.x = MAX_LINEAR_SPEED;
            if (cmd.linear.x < -MAX_LINEAR_SPEED) cmd.linear.x = -MAX_LINEAR_SPEED;
        }

        _cmd_pub.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_nav_node");
    SimpleNavNode node;
    node.run();
    return 0;
}