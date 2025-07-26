//
// Created by shiboz on 2021-02-06.
//

#include "arise_slam_mid360/LidarProcess/factor/pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

      p = _p + dp;
      q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
int PoseLocalParameterization::AmbientSize() const
{
  return 7;
}
int PoseLocalParameterization::TangentSize() const
{
  return 6;
}

// Dummy
bool PoseLocalParameterization::Minus(const double *x, const double *delta, double *x_minus_delta) const
{
  throw std::invalid_argument("SE3::Manifold::Minus() should never be called");
}

bool PoseLocalParameterization::MinusJacobian(const double* x, double* jacobian) const
{
  throw std::invalid_argument("SE3::Manifold::MinusJacobian() should never be called");
}