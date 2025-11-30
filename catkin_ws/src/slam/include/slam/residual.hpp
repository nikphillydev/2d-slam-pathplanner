#pragma once

#include "ros/ros.h"
#include "slam/DoubleOccupancyGrid.h"
#include "geometry_msgs/Point32.h"

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <vector>
#include <cmath>

// --- Helpers ---

// Extract scalar value from T (double or Jet)
template <typename T>
inline double GetScalar(const T& x) { return x; }

template <typename T, int N>
inline double GetScalar(const ceres::Jet<T, N>& x) { return x.a; }

// --- Residuals ---

struct PointToMapResidual {
    PointToMapResidual(const geometry_msgs::Point32& point, const slam::DoubleOccupancyGrid& map)
        : _point(point), _map(map) {}

    template <typename T>
    bool operator()(const T* const map_to_base, T* residual) const {
        // transformation map -> base_link
        T tf_x = map_to_base[0];
        T tf_y = map_to_base[1];
        T tf_theta = map_to_base[2];

        // point in base_link frame
        T p_x = static_cast<T>(_point.x);
        T p_y = static_cast<T>(_point.y);

        // map parameters
        T map_origin_x = static_cast<T>(_map.info.origin.position.x);
        T map_origin_y = static_cast<T>(_map.info.origin.position.y);
        T map_resolution = static_cast<T>(_map.info.resolution);
        T map_width = static_cast<T>(_map.info.width);
        T map_height = static_cast<T>(_map.info.height);

        // transform point in base_link frame into map frame
        T map_x = ceres::cos(tf_theta) * p_x - ceres::sin(tf_theta) * p_y + tf_x;
        T map_y = ceres::sin(tf_theta) * p_x + ceres::cos(tf_theta) * p_y + tf_y;

        // convert map coordinates to grid indices
        T map_x_idx = (map_x - map_origin_x) / map_resolution;
        T map_y_idx = (map_y - map_origin_y) / map_resolution;

        // boundary check (bicubic spline interpolation requires 2 neighbours)
        if (map_x_idx < T(2.0) || map_x_idx >= map_width - T(2) || 
            map_y_idx < T(2.0) || map_y_idx >= map_height - T(2)) 
        {
            // OUTSIDE of map
            // points outside of the map return a high residual
            // proportional to their distance from the map edge,
            // this results in pushing the optimizer back into the map

            // calculate distances to valid edges

            T dist_x = T(0.0);
            if (map_x_idx < T(2.0)) 
                dist_x = T(2.0) - map_x_idx;                // positive number
            else if (map_x_idx >= map_width - T(2))
                dist_x = map_x_idx - (map_width - T(2));    // positive number

            T dist_y = T(0.0);
            if (map_y_idx < T(2.0)) 
                dist_y = T(2.0) - map_y_idx;                // positive number
            else if (map_y_idx >= map_width - T(2))
                dist_y = map_y_idx - (map_width - T(2));    // positive number
            
            // create residual from high cost constant and distances to map edges
            // distance components ensure there is a gradient pointing back to the map
            residual[0] = T(1.0) + dist_x + dist_y;
        }
        else
        {
            // INSIDE of map
            T probability = GetBiCubicBSpline(map_x_idx, map_y_idx);
            residual[0] = T(1.0) - probability; 
        }
        return true;
    }

    // --- High Performance Cubic B-Spline Interpolation ---
    template <typename T>
    T GetBiCubicBSpline(const T& x, const T& y) const {
        const int w = _map.info.width;
        const int h = _map.info.height;
        int ix = static_cast<int>(std::floor(GetScalar(x)));
        int iy = static_cast<int>(std::floor(GetScalar(y)));

        // safe check for the raw array access
        if (ix < 1 || ix >= w - 2 || iy < 1 || iy >= h - 2) return T(0.0);

        T fx = x - T(ix);
        T fy = y - T(iy);

        // precompute powers
        T fx2 = fx * fx; T fx3 = fx2 * fx;
        T fy2 = fy * fy; T fy3 = fy2 * fy;

        // basis functions (weights)
        T wx0 = (T(1.0)-fx)*(T(1.0)-fx)*(T(1.0)-fx);
        T wx1 = T(3.0)*fx3 - T(6.0)*fx2 + T(4.0);
        T wx2 = T(-3.0)*fx3 + T(3.0)*fx2 + T(3.0)*fx + T(1.0);
        T wx3 = fx3;

        T wy0 = (T(1.0)-fy)*(T(1.0)-fy)*(T(1.0)-fy);
        T wy1 = T(3.0)*fy3 - T(6.0)*fy2 + T(4.0);
        T wy2 = T(-3.0)*fy3 + T(3.0)*fy2 + T(3.0)*fy + T(1.0);
        T wy3 = fy3;

        // pointer to the top-left corner of the 4x4 patch (x-1, y-1)
        const double* ptr = &_map.data[(iy - 1) * w + (ix - 1)];

        // lamba to parse unknown (-1) cells into 0.5 (neutral) probability
        auto val = [&](int idx) -> double {
            double v = ptr[idx];
            return (v < 0.0) ? 0.5 : v;
        };

        // convolve rows
        T row0 = T(val(0))*wx0 + T(val(1))*wx1 + T(val(2))*wx2 + T(val(3))*wx3; 
        T row1 = T(val(w))*wx0 + T(val(w+1))*wx1 + T(val(w+2))*wx2 + T(val(w+3))*wx3; 
        T row2 = T(val(2*w))*wx0 + T(val(2*w+1))*wx1 + T(val(2*w+2))*wx2 + T(val(2*w+3))*wx3; 
        T row3 = T(val(3*w))*wx0 + T(val(3*w+1))*wx1 + T(val(3*w+2))*wx2 + T(val(3*w+3))*wx3;

        // interpolate and normalize
        return (row0*wy0 + row1*wy1 + row2*wy2 + row3*wy3) * T(1.0/36.0);
    }

    const geometry_msgs::Point32& _point;
    const slam::DoubleOccupancyGrid& _map;
};