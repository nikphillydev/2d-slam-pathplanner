#pragma once

#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/jet.h>

// --- Helper Functions to handle both double and Jet types ---

// Case 1: T is a double (Value evaluation)
template <typename T>
inline double GetScalar(const T& x) {
    return x;
}

// Case 2: T is a Ceres Jet (Derivative evaluation)
template <typename T, int N>
inline double GetScalar(const ceres::Jet<T, N>& x) {
    return x.a;
}

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
        // pose[0] = x, pose[1] = y, pose[2] = theta

        const T& x = pose[0];
        const T& y = pose[1];
        const T& theta = pose[2];

        T cos_th = ceres::cos(theta);
        T sin_th = ceres::sin(theta);

        // 1. base_link → world/odom
        T trans_x = cos_th * T(point_x_) - sin_th * T(point_y_) + x;
        T trans_y = sin_th * T(point_x_) + cos_th * T(point_y_) + y;

        // 2. world → map
        T map_x = (trans_x - T(origin_x_)) / T(resolution_);
        T map_y = (trans_y - T(origin_y_)) / T(resolution_);

        // 3. Bi-Linear Interpolation
        T probability;
        bool valid = GetBiLinearInterpolation(map_x, map_y, probability);

        if (!valid) {
            // unknown
            residual[0] = T(0.0);
            return true;
        }

        // 4. Residual: 1.0 - probability of being occupied
        residual[0] = T(1.0) - probability; 

        return true;
    }

    template <typename T>
    bool GetBiLinearInterpolation(const T& x, const T& y, T& probability) const {
        // index
        double fx = GetScalar(x);
        double fy = GetScalar(y);

        int x0 = static_cast<int>(std::floor(fx));
        int y0 = static_cast<int>(std::floor(fy));
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        // 1. Boundary Check
        if (x0 < 0 || x1 >= width_ || y0 < 0 || y1 >= height_) {
            return false;
        }

        auto sample = [&](int ix, int iy) -> double {
            return map_data_[iy * width_ + ix];
        };

        double v00 = sample(x0, y0);
        double v10 = sample(x1, y0);
        double v01 = sample(x0, y1);
        double v11 = sample(x1, y1);

        if (v00 < 0.0 || v10 < 0.0 || v01 < 0.0 || v11 < 0.0) {
            return false;
        }

        double tx = fx - x0;
        double ty = fy - y0;

        double val_bottom = (1.0 - tx) * v00 + tx * v10;
        double val_top    = (1.0 - tx) * v01 + tx * v11;
        double val        = (1.0 - ty) * val_bottom + ty * val_top;

        probability = T(val);
        return true;
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
    // alpha_pos: weight for position (x, y)
    // alpha_rot: weight for rotation (theta)
    PosePriorResidual(double x_head, double y_head, double theta_head, 
                      double alpha_pos, double alpha_rot)
        : _x_head(x_head), _y_head(y_head), _theta_head(theta_head), 
          _sqrt_alpha_pos(std::sqrt(alpha_pos)), 
          _sqrt_alpha_rot(std::sqrt(alpha_rot)) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // pose[0]: x, pose[1]: y, pose[2]: theta
        
        // 1. Translation Error
        residuals[0] = T(_sqrt_alpha_pos) * (pose[0] - T(_x_head));
        residuals[1] = T(_sqrt_alpha_pos) * (pose[1] - T(_y_head));

        // 2. Rotation Error
        T theta_diff = pose[2] - T(_theta_head);

        residuals[2] = T(_sqrt_alpha_rot) * theta_diff;

        return true;
    }

    const double _x_head, _y_head, _theta_head;
    const double _sqrt_alpha_pos, _sqrt_alpha_rot;
};