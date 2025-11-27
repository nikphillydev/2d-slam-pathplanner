#pragma once

#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/jet.h>

// --- Helper: Extract scalar value from T (double or Jet) ---
template <typename T>
inline double GetScalar(const T& x) { return x; }

template <typename T, int N>
inline double GetScalar(const ceres::Jet<T, N>& x) { return x.a; }

// -----------------------------------------------------------

struct PointToMapResidual {
    PointToMapResidual(double x, double y, 
                       const std::vector<double>& map_data, 
                       int width, int height, double resolution,
                       double origin_x, double origin_y)
        : point_x_(x), point_y_(y), 
          map_data_(map_data), 
          width_(width), height_(height), resolution_(resolution),
          origin_x_(origin_x), origin_y_(origin_y) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // 1. Transform Point
        T theta = pose[2];
        T cos_th = ceres::cos(theta);
        T sin_th = ceres::sin(theta);

        T trans_x = cos_th * T(point_x_) - sin_th * T(point_y_) + pose[0];
        T trans_y = sin_th * T(point_x_) + cos_th * T(point_y_) + pose[1];

        // 2. Map Coordinates
        T map_x = (trans_x - T(origin_x_)) / T(resolution_);
        T map_y = (trans_y - T(origin_y_)) / T(resolution_);

        // 3. Interpolate (High Performance B-Spline)
        T probability = GetBiCubicBSpline(map_x, map_y);

        // 4. Residual
        // We want probability -> 1.0. 
        residual[0] = T(1.0) - probability; 

        return true;
    }

    // --- Optimized Cubic B-Spline Interpolation ---
    // Smoother than Catmull-Rom (reduces drift) and vectorized memory access (faster).
    template <typename T>
    T GetBiCubicBSpline(const T& x, const T& y) const {
        // 1. Floor to find the relevant 4x4 grid patch
        // B-Splines are centered at integer + 0.5, but for standard grids, 
        // we usually look at the index to the left.
        int ix = static_cast<int>(std::floor(GetScalar(x)));
        int iy = static_cast<int>(std::floor(GetScalar(y)));

        // 2. Fast Bounds Check (Check the whole 4x4 block at once)
        // We need (ix-1) to (ix+2)
        if (ix < 1 || ix >= width_ - 2 || iy < 1 || iy >= height_ - 2) {
            // Fallback: simple boundary check or return 0
            // Returning 0.0 with 0 gradient is safer than crashing
            return T(0.0);
        }

        // 3. Compute Weights (Uniform Cubic B-Spline Basis)
        // t is the fractional part relative to the center of the interval
        T fx = x - T(ix);
        T fy = y - T(iy);

        // Precompute powers for X
        T fx2 = fx * fx;
        T fx3 = fx2 * fx;
        
        // B-Spline weights for X (1/6 factor included at the end)
        // These are hardcoded basis functions for efficiency
        T wx0 = (T(1.0) - fx) * (T(1.0) - fx) * (T(1.0) - fx);      // (1-t)^3
        T wx1 = (T(3.0) * fx3) - (T(6.0) * fx2) + T(4.0);           // 3t^3 - 6t^2 + 4
        T wx2 = (T(-3.0) * fx3) + (T(3.0) * fx2) + (T(3.0) * fx) + T(1.0); // -3t^3 + 3t^2 + 3t + 1
        T wx3 = fx3;                                                // t^3

        // Precompute powers for Y
        T fy2 = fy * fy;
        T fy3 = fy2 * fy;
        
        // B-Spline weights for Y
        T wy0 = (T(1.0) - fy) * (T(1.0) - fy) * (T(1.0) - fy);
        T wy1 = (T(3.0) * fy3) - (T(6.0) * fy2) + T(4.0);
        T wy2 = (T(-3.0) * fy3) + (T(3.0) * fy2) + (T(3.0) * fy) + T(1.0);
        T wy3 = fy3;

        // 4. Fast Memory Access
        // Get pointer to the top-left corner of the 4x4 patch: (ix-1, iy-1)
        const double* ptr = &map_data_[(iy - 1) * width_ + (ix - 1)];

        // 5. Convolve (Matrix multiplication unrolled)
        // Row 0 (y-1)
        T row0 = (T(ptr[0]) * wx0 + T(ptr[1]) * wx1 + T(ptr[2]) * wx2 + T(ptr[3]) * wx3);
        ptr += width_; // Move down one row
        
        // Row 1 (y)
        T row1 = (T(ptr[0]) * wx0 + T(ptr[1]) * wx1 + T(ptr[2]) * wx2 + T(ptr[3]) * wx3);
        ptr += width_;

        // Row 2 (y+1)
        T row2 = (T(ptr[0]) * wx0 + T(ptr[1]) * wx1 + T(ptr[2]) * wx2 + T(ptr[3]) * wx3);
        ptr += width_;

        // Row 3 (y+2)
        T row3 = (T(ptr[0]) * wx0 + T(ptr[1]) * wx1 + T(ptr[2]) * wx2 + T(ptr[3]) * wx3);

        // Interpolate columns
        T val = (row0 * wy0 + row1 * wy1 + row2 * wy2 + row3 * wy3);

        // 6. Normalize (1/36 total because 1/6 for x and 1/6 for y)
        return val * T(1.0 / 36.0);
    }

    const double point_x_;
    const double point_y_;
    const std::vector<double>& map_data_; 
    const int width_;
    const int height_;
    const double resolution_;
    const double origin_x_;
    const double origin_y_;
};

struct PosePriorResidual {
    PosePriorResidual(double x_head, double y_head, double theta_head, double alpha_pos, double alpha_rot)
        : _x_head(x_head), _y_head(y_head), _theta_head(theta_head), 
          _sqrt_alpha_pos(std::sqrt(alpha_pos)), 
          _sqrt_alpha_rot(std::sqrt(alpha_rot)) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        residuals[0] = T(_sqrt_alpha_pos) * (pose[0] - T(_x_head));
        residuals[1] = T(_sqrt_alpha_pos) * (pose[1] - T(_y_head));
        
        // Note: For full robustness, normalize angle diff to [-pi, pi]
        residuals[2] = T(_sqrt_alpha_rot) * (pose[2] - T(_theta_head));
        return true;
    }

    const double _x_head, _y_head, _theta_head;
    const double _sqrt_alpha_pos, _sqrt_alpha_rot;
};