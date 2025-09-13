// UwbPoseRangeInterpFactor.h
#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * Auto-diff Ceres factor for one UWB range measurement,
 * interpolating between two U-frame poses T_k and T_{k+1}.
 *
 * Parameter blocks (each length 7):
 *   pose_k[0..2] = t_k (tx,ty,tz)
 *   pose_k[3..6] = q_k (qx,qy,qz,qw)  (must be unit quaternion)
 *
 * ctor args:
 *   anchor_pos    = a_j
 *   tag_offset    = δ_i
 *   measured_range= d_{ij}^m
 *   u             = interpolation ratio in [0,1]
 */
class UwbPoseRangeInterpFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UwbPoseRangeInterpFactor(const Eigen::Vector3d& anchor_pos,
                           const Eigen::Vector3d& tag_offset,
                           double measured_range,
                           double u)
    : anchor_(anchor_pos),
      tag_offset_(tag_offset),
      d_(measured_range),
      u_(u)
  {}

  template <typename T>
  bool operator()(const T* const pose_k,
                  const T* const pose_k1,
                  T* residual) const
  {
    // --- unpack poses ---
    Eigen::Matrix<T,3,1> p_k( pose_k[0], pose_k[1], pose_k[2] );
    Eigen::Quaternion<T> q_k( pose_k[6], pose_k[3], pose_k[4], pose_k[5] );
    Eigen::Matrix<T,3,1> p_k1( pose_k1[0], pose_k1[1], pose_k1[2] );
    Eigen::Quaternion<T> q_k1( pose_k1[6], pose_k1[3], pose_k1[4], pose_k1[5] );

    q_k.normalize();
    q_k1.normalize();

    // --- interpolate translation & rotation ---
    T t_u = T(u_);
    T one_minus_u = T(1.0) - t_u;
    Eigen::Matrix<T,3,1> p_m = one_minus_u * p_k + t_u * p_k1;
    // Eigen provides slerp
    Eigen::Quaternion<T> q_m = q_k.slerp(t_u, q_k1);

    // --- apply tag offset in U frame ---
    Eigen::Matrix<T,3,1> offset( T(tag_offset_.x()),
                                 T(tag_offset_.y()),
                                 T(tag_offset_.z()) );
    Eigen::Matrix<T,3,1> p_tag = q_m * offset;

    // --- compute range residual ---
    Eigen::Matrix<T,3,1> diff = p_m + p_tag - anchor_.cast<T>();
    residual[0] = diff.norm() - T(d_);
    return true;
  }

  /// Convenience to construct the AutoDiff cost function
  static ceres::CostFunction* Create(const Eigen::Vector3d& anchor_pos,
                                     const Eigen::Vector3d& tag_offset,
                                     double measured_range,
                                     double u)
  {
    return new ceres::AutoDiffCostFunction<
              UwbPoseRangeInterpFactor,      // functor
              1,                              // residual dimension
              7, 7                            // sizes of the two parameter blocks
           >(
           new UwbPoseRangeInterpFactor(anchor_pos, tag_offset, measured_range, u)
         );
  }

private:
  const Eigen::Vector3d anchor_;
  const Eigen::Vector3d tag_offset_;
  const double         d_;
  const double         u_;
};