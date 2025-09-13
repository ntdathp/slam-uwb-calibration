#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct RelOdomAutoDiffFactor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelOdomAutoDiffFactor(const Eigen::Vector3d& p_i,
                        const Eigen::Vector3d& p_j,
                        const Eigen::Quaterniond& q_i,
                        const Eigen::Quaterniond& q_j,
                        double p_sigma, double q_sigma)
      : delta_p_((q_i.inverse() * (p_j - p_i))),
        delta_q_(q_i.inverse() * q_j),
        inv_p_sigma_(1.0 / std::max(1e-9, p_sigma)),
        inv_q_sigma_(1.0 / std::max(1e-9, q_sigma)) {}

  template <typename T>
  bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
    // pose = [tx,ty,tz, qx,qy,qz,qw]
    Eigen::Map<const Eigen::Matrix<T,3,1>> Pi(pose_i + 0);
    Eigen::Quaternion<T> Qi(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);

    Eigen::Map<const Eigen::Matrix<T,3,1>> Pj(pose_j + 0);
    Eigen::Quaternion<T> Qj(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

    // relative in U
    Eigen::Matrix<T,3,1> r_pos = Qi.conjugate() * (Pj - Pi) - delta_p_.cast<T>();
    Eigen::Quaternion<T> q_err = delta_q_.conjugate().cast<T>() * (Qi.conjugate() * Qj);

    Eigen::Map<Eigen::Matrix<T,6,1>> r(residuals);
    r.template segment<3>(0) = T(inv_p_sigma_) * r_pos;
    r.template segment<3>(3) = T(2.0 * inv_q_sigma_) * q_err.vec();  // 2*vec(q_err)
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& p_i,
                                     const Eigen::Vector3d& p_j,
                                     const Eigen::Quaterniond& q_i,
                                     const Eigen::Quaterniond& q_j,
                                     double p_sigma, double q_sigma) {
    return new ceres::AutoDiffCostFunction<RelOdomAutoDiffFactor, 6, 7, 7>(
      new RelOdomAutoDiffFactor(p_i, p_j, q_i, q_j, p_sigma, q_sigma));
  }

  const Eigen::Vector3d    delta_p_;
  const Eigen::Quaterniond delta_q_;
  const double             inv_p_sigma_, inv_q_sigma_;
};