//
// Created by shiboz on 2021-02-06.
//

#ifndef ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H
#define ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H


#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../../utils/utility.h"
class PoseLocalParameterization : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool PlusJacobian(const double *x, double *jacobian) const;
    virtual int AmbientSize() const;
    virtual int TangentSize() const;

    virtual bool Minus(const double *x, const double *delta, double *x_minus_delta) const;
    virtual bool MinusJacobian(const double* x, double* jacobian) const;
};






#endif //ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H
