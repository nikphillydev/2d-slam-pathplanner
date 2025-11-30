#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Transform.h"

#include <vector>
#include <limits>

class PurePursuitController
{
public:
    PurePursuitController();
    ~PurePursuitController();

    void initialize_path(const std::vector<geometry_msgs::Point> path, const geometry_msgs::Transform& robot_tf);
    geometry_msgs::Twist follow_path(const geometry_msgs::Transform& robot_tf);

private:

    geometry_msgs::Point get_goal_point(const geometry_msgs::Transform& robot_tf);
    geometry_msgs::Twist get_velocity(const geometry_msgs::Transform& robot_tf, const geometry_msgs::Point& goal_point);

    int sgn(double num);
    double point_to_point_distance(double x1, double y1, double x2, double y2);

    std::vector<geometry_msgs::Point> _path;
    int _last_found_index;
    bool _approaching_final_point;

    // parameters

    const double LOOK_AHEAD_DISTANCE = 0.5;         // m
    const double FINAL_GOAL_TOLERANCE = 0.5;        // m
    const double KP_LINEAR = 1.0;
    const double KP_ANGULAR = 1.2;
    const double MAX_LINEAR_SPEED = 0.3;            // m / s
    const double MAX_ANGULAR_SPEED = 30;            // deg / s
    const double X_SOLN_TOLERANCE = 0.0001;         // m
    const double Y_SOLN_TOLERANCE = 0.0001;         // m
};