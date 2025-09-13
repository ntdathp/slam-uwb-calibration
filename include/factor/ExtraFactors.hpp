#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>

/**
 * PlaneFactor
 * Enforces that the anchor's z coordinate stays on a known horizontal plane.
 */
struct PlaneFactor {
  PlaneFactor() = default;

  template <typename T>
  bool operator()(const T *const anchor, T *residual) const {
    // Penalize only if z < 0 (i.e., use max(0, -z) as residual)
    residual[0] = ceres::fmax(T(0.0), -anchor[2]);
    return true;
  }
};

struct RangeFactor {
  RangeFactor(const Eigen::Vector3d& p_tag,
              const Eigen::Vector3d& p_anchor,
              double d)
    : p_tag_(p_tag), p_anchor_(p_anchor), d_(d) {}

  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    const T tx = x[0], ty = x[1], psi = x[2];
    const T c = ceres::cos(psi), s = ceres::sin(psi);

    const T tag_x = T(p_tag_.x()), tag_y = T(p_tag_.y()), tag_z = T(p_tag_.z());
    T px = c * tag_x - s * tag_y + tx;
    T py = s * tag_x + c * tag_y + ty;
    T pz = tag_z;

    T dx = px - T(p_anchor_.x()),
      dy = py - T(p_anchor_.y()),
      dz = pz - T(p_anchor_.z());
    T range = ceres::sqrt(dx*dx + dy*dy + dz*dz);
    residual[0] = range - T(d_);
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& p_tag,
                                     const Eigen::Vector3d& p_anchor,
                                     double d) {
    return new ceres::AutoDiffCostFunction<RangeFactor,1,3>(
      new RangeFactor(p_tag, p_anchor, d));
  }

private:
  const Eigen::Vector3d p_tag_, p_anchor_;
  const double d_;
};