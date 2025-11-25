#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <cmath>

inline double ExtractScalar(double value) {
  return value;
}

template <typename S, int N>
double ExtractScalar(const ceres::Jet<S, N>& value) {
  return static_cast<double>(value.a);
}

// Msmooth (bicubic interpolation)
double cubicInterpolate(double p0, double p1, double p2, double p3, double x) {
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 +
      x * (3.0 * (p1 - p2) + p3 - p0)));
}
double getGridValue(const std::vector<std::vector<double>>& grid, int x, int y) {
  int width = grid[0].size();
  int height = grid.size();
  if (x < 0) x = 0;
  if (x >= width) x = width - 1;
  if (y < 0) y = 0;
  if (y >= height) y = height - 1;
  return grid[y][x];
}

double Msmooth(const std::vector<std::vector<double>>& grid, double x, double y) {
    int ix = static_cast<int>(std::floor(x));
    int iy = static_cast<int>(std::floor(y));
    double fx = x - ix;
    double fy = y - iy;

    // cubic interpolation in x direction
    double col[4];
    for (int j = -1; j <= 2; ++j) {
        double p0 = getGridValue(grid, ix-1, iy+j);
        double p1 = getGridValue(grid, ix,   iy+j);
        double p2 = getGridValue(grid, ix+1, iy+j);
        double p3 = getGridValue(grid, ix+2, iy+j);
        col[j+1] = cubicInterpolate(p0, p1, p2, p3, fx);
    }

    // cubic interpolation in y direction
    double value = cubicInterpolate(col[0], col[1], col[2], col[3], fy);
    return value;
}

struct ScanMatchCostFunctor{
    Eigen::Vector2d hk_;
    const std::vector<std::vector<double>> *grid_;

  ScanMatchCostFunctor(const Eigen::Vector2d& hk,
             const std::vector<std::vector<double>> *grid)
    : hk_(hk), grid_(grid) {}
    
    template <typename T>
    bool operator()(const T* const xi, T* residual) const {
        const T& x = xi[0];
        const T& y = xi[1];
        const T& theta = xi[2];
        // coordinate transformation
        T cos_t = ceres::cos(theta);
        T sin_t = ceres::sin(theta);
        T hx = T(hk_.x());
        T hy = T(hk_.y());
        T wx = x + cos_t * hx - sin_t * hy;
        T wy = y + sin_t * hx + cos_t * hy;
        
        double wx_d = ExtractScalar(wx);
        double wy_d = ExtractScalar(wy);
        double m_d = Msmooth(*grid_, wx_d, wy_d);
        T m = T(m_d);
        residual[0] = T(1.0) - m;
        return true;
    }
};



