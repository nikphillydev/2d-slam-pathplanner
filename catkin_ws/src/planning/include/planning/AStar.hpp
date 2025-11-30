#pragma once

#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <algorithm>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"


// A Node in the A* algorithm
struct Node {
    int x, y;
    int index;
    double g;
    double h;
    double f;
    std::shared_ptr<Node> parent;

    Node(int x_, int y_, int index_, double g_, double h_, double f_, std::shared_ptr<Node> parent_ = nullptr)
        : x(x_), y(y_), index(index_), g(g_), h(h_), f(f_), parent(parent_) {}
};

// Comparator for the priority queue
struct CompareNode {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f > b->f;
    }
};


class AStar {
    public:
    AStar();
    ~AStar();

    bool make_plan(const geometry_msgs::Point& start, const geometry_msgs::Point& goal, const nav_msgs::OccupancyGrid& original_map, std::vector<geometry_msgs::Point>& path);

    private:
    const double OCCUPANCY_THRESHOLD = 50;  
    const double INFLATION_RADIUS = 0.3;    // meters

    double calculate_heuristic(int x1, int y1, int x2, int y2);

    // helper
    int grid_to_index(int x, int y, int width);
    void index_to_grid(int index, int width, int& x, int& y);
    void grid_to_world(int x, int y, const nav_msgs::MapMetaData& info, geometry_msgs::Point& world_pt);
    void world_to_grid(double x, double y, const nav_msgs::MapMetaData& info, int& grid_x, int& grid_y);
    bool is_valid(int x, int y, const nav_msgs::OccupancyGrid& map);

    // preprocess the map
    void inflate_obstacles(nav_msgs::OccupancyGrid& map);
    void undetect_to_free(nav_msgs::OccupancyGrid& map);

};





