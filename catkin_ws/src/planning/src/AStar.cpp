#include "planning/AStar.hpp"


AStar::AStar(){}
AStar::~AStar(){}

bool AStar::make_plan(const geometry_msgs::Point& start, const geometry_msgs::Point& goal, const nav_msgs::OccupancyGrid& origin_map, std::vector<geometry_msgs::Point>& path) {
    path.clear();
    nav_msgs::OccupancyGrid map = origin_map;
    // 1. Preprocess the map TO BE DELETED
    inflate_obstacles(map);
    // undetect_to_free(map);

    // 2. Convert world coordinates to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    world_to_grid(start.x, start.y, map.info, start_x, start_y);
    world_to_grid(goal.x, goal.y, map.info, goal_x, goal_y);

    if (!is_valid(goal_x, goal_y, map)) {
        ROS_WARN("A*: Goal position is in an obstacle/invalid.");
        return false;
    }

    // 3. Init data structures
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list;
    std::vector<bool> closed_list(map.info.width * map.info.height, false);
    std::vector<double> g_score(map.info.width * map.info.height, std::numeric_limits<double>::infinity());

    // 4. Setup start node
    int start_index = grid_to_index(start_x, start_y, map.info.width);
    double start_h = calculate_heuristic(start_x, start_y, goal_x, goal_y);
    auto start_node = std::make_shared<Node>(start_x, start_y, start_index, 0.0, start_h, 0.0 + start_h, nullptr);
    open_list.push(start_node);
    g_score[start_index] = 0.0;

    // 5. Main loop
    while(!open_list.empty()){
        ROS_INFO("Open list size: %lu", open_list.size());
        auto current = open_list.top();
        open_list.pop();
        if (closed_list[current->index])
            continue;
        closed_list[current->index] = true;
        
        if (std::sqrt(std::pow(current->x - goal_x, 2) + std::pow(current->y - goal_y, 2)) < STRIDE_CELLS) {
            // Found path. Reconstruct path
            auto step = current;
            while (step != nullptr) {
                geometry_msgs::Point world_pt;
                grid_to_world(step->x, step->y, map.info, world_pt);
                path.push_back(world_pt);
                step = step->parent;
            }
            std::reverse(path.begin(), path.end());
            if (path.size() > 4) {
                smooth_path(path); 
            }
            return true;
        }

        // Find neighbors and update scores
        const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
        const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
        const double costs[] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};
        
        

        for (int i = 0; i < 8; ++i) {
            int nx = current->x + STRIDE_CELLS *dx[i];
            int ny = current->y + STRIDE_CELLS *dy[i];
            if (!is_valid(nx, ny, map))
                continue;

            int neighbor_index = grid_to_index(nx, ny, map.info.width);
            if (closed_list[neighbor_index])
                continue;

            double tentative_g = current->g + STRIDE_CELLS * costs[i];

            if (tentative_g < g_score[neighbor_index]) {
                g_score[neighbor_index] = tentative_g;
                double h = calculate_heuristic(nx, ny, goal_x, goal_y);
                auto neighbor_node = std::make_shared<Node>(nx, ny, neighbor_index, tentative_g, h, tentative_g + h, current);
                open_list.push(neighbor_node);
            }
        }
    }
    ROS_WARN("A*: No path found.");
    return false;
}

double AStar::calculate_heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int AStar::grid_to_index(int x, int y, int width) {
    return y * width + x;
}

void AStar::index_to_grid(int index, int width, int& x, int& y) {
    y = index / width;
    x = index % width;
}

void AStar::grid_to_world(int x, int y, const nav_msgs::MapMetaData& info, geometry_msgs::Point& world_pt) {
    world_pt.x = info.origin.position.x + (x + 0.5) * info.resolution;
    world_pt.y = info.origin.position.y + (y + 0.5) * info.resolution;
    world_pt.z = 0.0;
}

void AStar::world_to_grid(double x, double y, const nav_msgs::MapMetaData& info, int& grid_x, int& grid_y) {
    grid_x = static_cast<int>((x - info.origin.position.x) / info.resolution);
    grid_y = static_cast<int>((y - info.origin.position.y) / info.resolution);
}

bool AStar::is_valid(int x, int y, const nav_msgs::OccupancyGrid& map) {
    if (x < 0 || x >= map.info.width || y < 0 || y >= map.info.height)
        return false;
    int index = grid_to_index(x, y, map.info.width);
    if (map.data[index] > OCCUPANCY_THRESHOLD)
        return false;
    return true;
}

//preprocess the map
//This may cost some time...
void AStar::inflate_obstacles(nav_msgs::OccupancyGrid& map){
    ros::Time start;
    start = ros::Time::now();
    int h = map.info.height;
    int w = map.info.width;
    int radius_cells = std::ceil(INFLATION_RADIUS / map.info.resolution);
    int r2 = radius_cells * radius_cells;
    std::vector<int8_t> inflated_data = map.data;
#pragma omp parallel for collapse(2)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (map.data[grid_to_index(x, y, w)] > OCCUPANCY_THRESHOLD) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            if (dx*dx + dy*dy <= r2) {
                                inflated_data[grid_to_index(nx, ny, w)] = 100;
                            }
                        }
                    }
                }
            }
        }
    }
    map.data.swap(inflated_data);
    ros::Time end = ros::Time::now();
    ROS_INFO("Inflation took: %f seconds", (end - start).toSec());
}

// Gradient Descent Smoothing
void AStar::smooth_path(std::vector<geometry_msgs::Point>& path) {
    double weight_data = 0.5;
    double weight_smooth = 0.35; 
    double tolerance = 0.00001;
    
    if (path.size() <= 4) return;

    std::vector<geometry_msgs::Point> new_path = path;
    
    double change = tolerance;
    int max_iterations = 1000;
    int iter = 0;

    while (change >= tolerance && iter < max_iterations) {
        change = 0.0;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double aux_x = new_path[i].x;
            double aux_y = new_path[i].y;

            new_path[i].x += weight_data * (path[i].x - new_path[i].x) + 
                             weight_smooth * (new_path[i-1].x + new_path[i+1].x - 2.0 * new_path[i].x);
            
            new_path[i].y += weight_data * (path[i].y - new_path[i].y) + 
                             weight_smooth * (new_path[i-1].y + new_path[i+1].y - 2.0 * new_path[i].y);

            change += std::abs(aux_x - new_path[i].x) + std::abs(aux_y - new_path[i].y);
        }
        iter++;
    }

    path = new_path;
}