#include "planning/pure_pursuit.hpp"

PurePursuitController::PurePursuitController()
{
}

PurePursuitController::~PurePursuitController()
{
}

void PurePursuitController::initialize_path(const std::vector<geometry_msgs::Point> path, const geometry_msgs::Transform& robot_tf)
{
    _path = path;
    _last_found_index = 0;
    _approaching_final_point = false;

    double robot_x = robot_tf.translation.x;
    double robot_y = robot_tf.translation.y;

    // find the closest point on the path at the first local minimum
    
    double closest_dist = std::numeric_limits<double>::max();
    int search_window = 10;          // allow checking 10 points ahead after a minimum is found
    int points_since_min = 0;

    for (int i = 0; i < _path.size(); i ++)
    {
        double new_dist = point_to_point_distance(robot_x, robot_y, _path[i].x, _path[i].y);

        if (new_dist < closest_dist)
        {
            // new point is closer to robot
            closest_dist = new_dist;
            _last_found_index = i;
            points_since_min = 0;       // reset patience counter
        }
        else
        {
            // distance is increasing or staying the same
            points_since_min++;
            if (points_since_min > search_window)
            {
                // consistently moved away for search_window steps
                break;
            }
        }
    }
}

geometry_msgs::Twist PurePursuitController::follow_path(const geometry_msgs::Transform& robot_tf)
{
    // get the goal point on the path
    geometry_msgs::Point goal_point = get_goal_point(robot_tf);

    // check if we are approaching final point on path
    if (_approaching_final_point)
    {
        double robot_x = robot_tf.translation.x;
        double robot_y = robot_tf.translation.y;
        if (point_to_point_distance(robot_x, robot_y, goal_point.x, goal_point.y))
        {
            geometry_msgs::Twist null_twist;
            return null_twist;
        }
    }

    // compute necessary robot velocity to follow goal point
    geometry_msgs::Twist twist = get_velocity(robot_tf, goal_point);

    return twist;
}

geometry_msgs::Point PurePursuitController::get_goal_point(const geometry_msgs::Transform& robot_tf)
{
    // initialize goal point to robot position 
    geometry_msgs::Point goal_point;
    goal_point.x = robot_tf.translation.x;
    goal_point.y = robot_tf.translation.y;
    goal_point.z = robot_tf.translation.z;

    if (_path.size() == 0)
    {
        return goal_point;
    }
    else if (_path.size() == 1)
    {
        goal_point = _path[0];
        return goal_point;
    }

    double robot_x = robot_tf.translation.x;
    double robot_y = robot_tf.translation.y;

    bool found_goal_point = false;

    // path contains more than 1 point, find goal point using circle-line intersection ...

    for (int i = _last_found_index; i < _path.size() - 1; i++)
    {
        // extract points on the path that will create the line: (x1, y1) and (x2, y2)

        double x1 = _path[i].x;
        double y1 = _path[i].y;
        double x2 = _path[i + 1].x;
        double y2 = _path[i + 1].y;

        // find points of intersection between circle centered at the robot with 
        // radius LOOK_AHEAD_DISTANCE and the infinite line drawn between (x1, y1) and (x2, y2)

        // apply robot offset to "center" the circle at (0,0)
        // this the simplifies the following math

        double x1_offset = x1 - robot_x;
        double y1_offset = y1 - robot_y;
        double x2_offset = x2 - robot_x;
        double y2_offset = y2 - robot_y;

        double dx = x2_offset - x1_offset;
        double dy = y2_offset - y1_offset;
        double dr = std::hypot(dx, dy);
        double D = x1_offset * y2_offset - x2_offset * y1_offset;
        double discriminant = LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE * dr * dr - D * D;

        if (discriminant < 0)
        {
            // discriminant is negative, therefore no intersection exists between circle and line
            break;
        }
        else
        {
            // an intersection exists; it is either TANGENT (1 real point) or SECANT (2 real points)

            double x1_sol = (D * dy + sgn(dy) * dx * std::sqrt(discriminant)) / (dr * dr) + robot_x;
            double x2_sol = (D * dy - sgn(dy) * dx * std::sqrt(discriminant)) / (dr*dr) + robot_x;
            double y1_sol = (-D * dx + std::abs(dy) * std::sqrt(discriminant)) / (dr*dr) + robot_y;
            double y2_sol = (-D * dx - std::abs(dy) * std::sqrt(discriminant)) / (dr*dr) + robot_y;

            // calculate min / max ranges to check if solutions are between (x1, y1) and (x2, y2)
            // recall: math assumed infinite line going through (x1, y1) and (x2, y2)
            double min_x = std::min(x1, x2) - X_SOLN_TOLERANCE;
            double max_x = std::max(x1, x2) + X_SOLN_TOLERANCE;
            double min_y = std::min(y1, y2) - Y_SOLN_TOLERANCE;
            double max_y = std::max(y1, y2) + Y_SOLN_TOLERANCE;

            // check if valid solution exists, i.e. a solution between (x1, y1) and (x2, y2)

            if ((min_x <= x1_sol && x1_sol <= max_x && min_y <= y1_sol && y1_sol <= max_y) || 
                (min_x <= x2_sol && x2_sol <= max_x && min_y <= y2_sol && y2_sol <= max_y))
            {
                // at least one valid solution exists ...

                if ((min_x <= x1_sol <= max_x && min_y <= y1_sol <= max_y) && (min_x <= x2_sol <= max_x and min_y <= y2_sol <= max_y))
                {
                    // both solutions are valid, choose the one closer to next point (x2, y2)

                    if (point_to_point_distance(x1_sol, y1_sol, x2, y2) < point_to_point_distance(x2_sol, y2_sol, x2, y2))
                    {
                        goal_point.x = x1_sol;
                        goal_point.y = y1_sol;
                    }
                    else
                    {
                        goal_point.x = x2_sol;
                        goal_point.y = y2_sol;
                    }
                    // valid goal point found
                    _last_found_index = i;
                    found_goal_point = true;
                    break;
                }
                else
                {
                    // only one solution is valid

                    if (min_x <= x1_sol <= max_x && min_y <= y1_sol <= max_y)
                    {
                        goal_point.x = x1_sol;
                        goal_point.y = y1_sol;
                    }
                    else
                    {
                        goal_point.x = x2_sol;
                        goal_point.y = y2_sol;
                    }

                    // edge case: check if robot is closer to (x2, y2) than valid solution (i.e. the goal point)
                    // in this case, update to the next line on path and re-run solution
                    if (point_to_point_distance(robot_x, robot_y, x2, y2) < point_to_point_distance(goal_point.x, goal_point.y, x2, y2))
                    {
                        // robot closer to (x2, y2)
                        _last_found_index = i + 1;
                        continue;
                    }

                    // valid goal point found
                    _last_found_index = i;
                    found_goal_point = true;
                    break;
                }
            }
            else
            {
                // check if line is fully enclosed in circle
                if ((point_to_point_distance(robot_x, robot_y, x1, y1) < LOOK_AHEAD_DISTANCE) && point_to_point_distance(robot_x, robot_y, x2, y2) < LOOK_AHEAD_DISTANCE)
                {
                    _last_found_index = i + 1;
                    continue;
                }

                // no valid solutions exist within range
                break;
            }
        }
    }

    // check if we are approaching final point on path
    if (_last_found_index == _path.size() - 1)
    {
        goal_point = _path[_path.size() - 1];
        found_goal_point = true;
        _approaching_final_point = true;
    }

    if (!found_goal_point)
    {
        // no valid goal point on path within LOOK_AHEAD_DISTANCE away from robot
        // therefore go to last point seen on path
        goal_point.x = _path[_last_found_index].x;
        goal_point.y = _path[_last_found_index].y;
    }

    return goal_point;
}

geometry_msgs::Twist PurePursuitController::get_velocity(const geometry_msgs::Transform& robot_tf, const geometry_msgs::Point& goal_point)
{
    geometry_msgs::Twist twist;

    // linear speed
    
    double dx = goal_point.x - robot_tf.translation.x;
    double dy = goal_point.y - robot_tf.translation.y;
    double distance = std::hypot(dx, dy);

    double linear_speed = distance * KP_LINEAR;
    if (linear_speed > MAX_LINEAR_SPEED) linear_speed = MAX_LINEAR_SPEED;
    
    // angular speed
    
    tf2::Quaternion q;
    tf2::fromMsg(robot_tf.rotation, q);
    double current_yaw = tf2::getYaw(q) * (180 / M_PI);

    double target_yaw = std::atan2(dy, dx) * (180 / M_PI);
    if (target_yaw < 0) target_yaw += 360;

    double yaw_error = target_yaw - current_yaw;
    if (yaw_error > 180 || yaw_error < -180)
    {
        yaw_error = -1 * sgn(yaw_error) * (360 - std::abs(yaw_error));
    }

    double angular_speed = yaw_error * KP_ANGULAR;

    if (angular_speed > MAX_ANGULAR_SPEED) angular_speed = MAX_ANGULAR_SPEED;
    if (angular_speed < -MAX_ANGULAR_SPEED) angular_speed = -MAX_ANGULAR_SPEED;

    // set speeds

    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed * (M_PI / 180);

    return twist;
}

int PurePursuitController::sgn(double num)
{
    if (num >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

double PurePursuitController::point_to_point_distance(double x1, double y1, double x2, double y2)
{
    double x_term = (x1 - x2) * (x1 - x2);
    double y_term = (y1 - y2) * (y1 - y2);
    return std::sqrt(x_term + y_term);
}