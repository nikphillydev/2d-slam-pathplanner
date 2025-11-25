#pragma once

#include <vector>
#include <ceres/ceres.h>
#include <ceres/jet.h>

// --- Helper Functions to handle both double and Jet types ---

// Case 1: T is a double (Value evaluation)
template <typename T>
inline double GetScalar(const T& x) {
    return x;
}

// Case 2: T is a Ceres Jet (Derivative evaluation)
// We extract '.a' which represents the scalar value of the variable
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

        T theta = pose[2];
        T cos_th = ceres::cos(theta);
        T sin_th = ceres::sin(theta);

        // Transform point to World Frame
        T trans_x = cos_th * T(point_x_) - sin_th * T(point_y_) + pose[0];
        T trans_y = sin_th * T(point_x_) + cos_th * T(point_y_) + pose[1];

        // Convert to Map Indices
        T map_x = (trans_x - T(origin_x_)) / T(resolution_);
        T map_y = (trans_y - T(origin_y_)) / T(resolution_);

        // Perform Interpolation
        T probability = GetBiLinearInterpolation(map_x, map_y);

        // Residual = 1.0 - Probability (We want to maximize probability)
        residual[0] = T(1.0) - probability; 

        return true;
    }

    template <typename T>
    T GetBiLinearInterpolation(const T& x, const T& y) const {
        // Use the helper to cast to integer indices
        // We cannot differentiate through the Index selection (it's a step function),
        // so we use the scalar value to find the cell.
        int x0 = (int)GetScalar(x);
        int y0 = (int)GetScalar(y);
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        // Check Bounds
        if (x0 < 0 || x1 >= width_ || y0 < 0 || y1 >= height_) {
            return T(0.0);
        }

        // Fetch Map Values (Probability 0.0 to 1.0)
        double v00 = map_data_[y0 * width_ + x0];
        double v10 = map_data_[y0 * width_ + x1];
        double v01 = map_data_[y1 * width_ + x0];
        double v11 = map_data_[y1 * width_ + x1];

        // Calculate Weights (These ARE differentiable)
        T wa = (T(x1) - x);
        T wb = (x - T(x0));
        T wc = (y - T(y0)); // Top weight
        T wd = (T(y1) - y); // Bottom weight

        // Bilinear Interpolation Formula
        // Interpolate X at bottom (y0) and top (y1)
        T val_bottom = T(v00) * wa + T(v10) * wb;
        T val_top    = T(v01) * wa + T(v11) * wb;

        // Interpolate Y
        return val_bottom * wd + val_top * wc;
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