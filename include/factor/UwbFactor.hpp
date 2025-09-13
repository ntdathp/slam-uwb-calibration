#ifndef FACTOR_UWBFACTOR_HPP
#define FACTOR_UWBFACTOR_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>

/**
 * UwbFactor
 *
 * This factor models the UWB ranging measurement between a tag and an anchor.
 * The residual is defined as:
 *
 *    residual = ||anchor - tag_position|| - measured_range
 *
 * where:
 *   - anchor: 3D position parameter (double[3])
 *   - tag_position: the effective position of the tag (obtained via odom+B-spline interpolation)
 *   - measured_range: the distance measured by the UWB sensor
 */
class UwbFactor {
public:
  UwbFactor(const Eigen::Vector3d& tag_pos, double dist)
      : tag_pos_(tag_pos), dist_(dist) {}
  template <typename T>
  bool operator()(const T* const anchor, T* residual) const {
    Eigen::Matrix<T, 3, 1> ap(anchor[0], anchor[1], anchor[2]);
    residual[0] = (ap - tag_pos_.cast<T>()).norm() - T(dist_);
    return true;
  }
private:
  Eigen::Vector3d tag_pos_;
  double dist_;
};
#endif  // FACTOR_UWBFACTOR_HPP