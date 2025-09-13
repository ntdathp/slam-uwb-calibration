#ifndef FACTOR_ANCHORDISTANCEFACTOR_HPP
#define FACTOR_ANCHORDISTANCEFACTOR_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>

/**
 * AnchorDistanceFactor
 *
 * This factor enforces the known distance between two anchors.
 *
 * The residual is defined as:
 *
 *    residual = ||anchor_i - anchor_j|| - known_distance
 *
 * where:
 *   - anchor_i: 3D position parameter of the first anchor (double[3])
 *   - anchor_j: 3D position parameter of the second anchor (double[3])
 *   - known_distance: the pre-measured distance between the two anchors
 */
struct AnchorDistanceFactor {
  AnchorDistanceFactor(double known_distance) : known_distance_(known_distance) {}

  template <typename T>
  bool operator()(const T* const anchor_i, const T* const anchor_j, T* residual) const {
    T dx = anchor_i[0] - anchor_j[0];
    T dy = anchor_i[1] - anchor_j[1];
    T dz = anchor_i[2] - anchor_j[2];
    T distance = ceres::sqrt(dx * dx + dy * dy + dz * dz);

    residual[0] = distance - T(known_distance_);
    return true;
  }

private:
  const double known_distance_;
};

#endif  // FACTOR_ANCHORDISTANCEFACTOR_HPP
