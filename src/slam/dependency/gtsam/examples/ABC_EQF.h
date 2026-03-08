/**
 * @file ABC_EQF.h
 * @brief Header file for the Attitude-Bias-Calibration Equivariant Filter
 *
 * This file contains declarations for the Equivariant Filter (EqF) for attitude
 * estimation with both gyroscope bias and sensor extrinsic calibration, based
 * on the paper: "Overcoming Bias: Equivariant Filter Design for Biased Attitude
 * Estimation with Online Calibration" by Fornasier et al. Authors: Darshan
 * Rajasekaran & Jennifer Oum
 */

#ifndef ABC_EQF_H
#define ABC_EQF_H
#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>  // For std::accumulate
#include <string>
#include <vector>

#include "ABC.h"

// All implementations are wrapped in this namespace to avoid conflicts
namespace gtsam {
namespace abc_eqf_lib {

using namespace std;
using namespace gtsam;

//========================================================================
// Helper Functions for EqF
//========================================================================

/// Calculate numerical differential

Matrix numericalDifferential(std::function<Vector(const Vector&)> f,
                             const Vector& x);

/**
 * Compute the lift of the system (Theorem 3.8, Equation 7)
 * @param xi State
 * @param u Input
 * @return Lift vector
 */
template <size_t N>
Vector lift(const State<N>& xi, const Input& u);

/**
 * Action of the symmetry group on the state space (Equation 4)
 * @param X Group element
 * @param xi State
 * @return New state after group action
 */
template <size_t N>
State<N> operator*(const G<N>& X, const State<N>& xi);

/**
 * Action of the symmetry group on the input space (Equation 5)
 * @param X Group element
 * @param u Input
 * @return New input after group action
 */
template <size_t N>
Input velocityAction(const G<N>& X, const Input& u);

/**
 * Action of the symmetry group on the output space (Equation 6)
 * @param X Group element
 * @param y Direction measurement
 * @param idx Calibration index
 * @return New direction after group action
 */
template <size_t N>
Vector3 outputAction(const G<N>& X, const Unit3& y, int idx);

/**
 * Differential of the phi action at E = Id in local coordinates
 * @param xi State
 * @return Differential matrix
 */
template <size_t N>
Matrix stateActionDiff(const State<N>& xi);

//========================================================================
// Equivariant Filter (EqF)
//========================================================================

/// Equivariant Filter (EqF) implementation
template <size_t N>
class EqF {
 private:
  int dof;                // Degrees of freedom
  G<N> X_hat;             // Filter state
  Matrix Sigma;           // Error covariance
  State<N> xi_0;          // Origin state
  Matrix Dphi0;           // Differential of phi at origin
  Matrix InnovationLift;  // Innovation lift matrix

  /**
   * Return the state matrix A0t (Equation 14a)
   * @param u Input
   * @return State matrix A0t
   */
  Matrix stateMatrixA(const Input& u) const;

  /**
   * Return the state transition matrix Phi (Equation 17)
   * @param u Input
   * @param dt Time step
   * @return State transition matrix Phi
   */
  Matrix stateTransitionMatrix(const Input& u, double dt) const;

  /**
   * Return the Input matrix Bt
   * @return Input matrix Bt
   */
  Matrix inputMatrixBt() const;

  /**
   * Return the measurement matrix C0 (Equation 14b)
   * @param d Known direction
   * @param idx Calibration index
   * @return Measurement matrix C0
   */
  Matrix measurementMatrixC(const Unit3& d, int idx) const;

  /**
   * Return the measurement output matrix Dt
   * @param idx Calibration index
   * @return Measurement output matrix Dt
   */
  Matrix outputMatrixDt(int idx) const;

 public:
  /**
   * Initialize EqF
   * @param Sigma Initial covariance
   * @param m Number of sensors
   */
  EqF(const Matrix& Sigma, int m);

  /**
   * Return estimated state
   * @return Current state estimate
   */
  State<N> stateEstimate() const;

  /**
   * Propagate the filter state
   * @param u Angular velocity measurement
   * @param dt Time step
   */
  void propagation(const Input& u, double dt);

  /**
   * Update the filter state with a measurement
   * @param y Direction measurement
   */
  void update(const Measurement& y);
};

//========================================================================
// Helper Functions Implementation
//========================================================================

/**
 * Maps system dynamics to the symmetry group
 * @param xi State
 * @param u Input
 * @return Lifted input in Lie Algebra
 * Uses Vector zero & Rot3 inverse, matrix functions
 */
template <size_t N>
Vector lift(const State<N>& xi, const Input& u) {
  Vector L = Vector::Zero(6 + 3 * N);

  // First 3 elements
  L.head<3>() = u.w - xi.b;

  // Next 3 elements
  L.segment<3>(3) = -u.W() * xi.b;

  // Remaining elements
  for (size_t i = 0; i < N; i++) {
    L.segment<3>(6 + 3 * i) = xi.S[i].inverse().matrix() * L.head<3>();
  }

  return L;
}
/**
 * Implements group actions on the states
 * @param X A symmetry group element G consisting of the attitude, bias and the
 * calibration components X.a -> Rotation matrix containing the attitude X.b ->
 * A skew-symmetric matrix representing bias X.B -> A vector of Rotation
 * matrices for the calibration components
 * @param xi State object
 * xi.R -> Attitude (Rot3)
 * xi.b -> Gyroscope Bias(Vector 3)
 * xi.S -> Vector of calibration matrices(Rot3)
 * @return Transformed state
 * Uses the Rot3 inverse and Vee functions
 */
template <size_t N>
State<N> operator*(const G<N>& X, const State<N>& xi) {
  std::array<Rot3, N> new_S;

  for (size_t i = 0; i < N; i++) {
    new_S[i] = X.A.inverse() * xi.S[i] * X.B[i];
  }

  return State<N>(xi.R * X.A, X.A.inverse().matrix() * (xi.b - Rot3::Vee(X.a)),
                  new_S);
}
/**
 * Transforms the angular velocity measurements b/w frames
 * @param X A symmetry group element X with the components
 * @param u Inputs
 * @return Transformed inputs
 * Uses Rot3 Inverse, matrix and Vee functions and is critical for maintaining
 * the input equivariance
 */
template <size_t N>
Input velocityAction(const G<N>& X, const Input& u) {
  return Input{X.A.inverse().matrix() * (u.w - Rot3::Vee(X.a)), u.Sigma};
}
/**
 * Transforms the Direction measurements based on the calibration type ( Eqn 6)
 * @param X Group element X
 * @param y Direction measurement y
 * @param idx Calibration index
 * @return Transformed direction
 * Uses Rot3 inverse, matric and Unit3 unitvector functions
 */
template <size_t N>
Vector3 outputAction(const G<N>& X, const Unit3& y, int idx) {
  if (idx == -1) {
    return X.A.inverse().matrix() * y.unitVector();
  } else {
    if (idx >= static_cast<int>(N)) {
      throw std::out_of_range("Calibration index out of range");
    }
    return X.B[idx].inverse().matrix() * y.unitVector();
  }
}

/**
 * @brief Calculates the Jacobian matrix using central difference approximation
 * @param f Vector function f
 * @param x The point at which Jacobian is evaluated
 * @return Matrix containing numerical partial derivatives of f at x
 * Uses Vector's size() and Zero(), Matrix's Zero() and col() methods
 */
Matrix numericalDifferential(std::function<Vector(const Vector&)> f,
                             const Vector& x) {
  double h = 1e-6;
  Vector fx = f(x);
  int n = fx.size();
  int m = x.size();
  Matrix Df = Matrix::Zero(n, m);

  for (int j = 0; j < m; j++) {
    Vector ej = Vector::Zero(m);
    ej(j) = 1.0;

    Vector fplus = f(x + h * ej);
    Vector fminus = f(x - h * ej);

    Df.col(j) = (fplus - fminus) / (2 * h);
  }

  return Df;
}

/**
 * Computes the differential of a state action at the identity of the symmetry
 * group
 * @param xi State object Xi representing the point at which to evaluate the
 * differential
 * @return A matrix representing the jacobian of the state action
 * Uses numericalDifferential, and Rot3 expmap, logmap
 */
template <size_t N>
Matrix stateActionDiff(const State<N>& xi) {
  std::function<Vector(const Vector&)> coordsAction = [&xi](const Vector& U) {
    G<N> groupElement = G<N>::exp(U);
    State<N> transformed = groupElement * xi;
    return xi.localCoordinates(transformed);
  };

  Vector zeros = Vector::Zero(6 + 3 * N);
  Matrix differential = numericalDifferential(coordsAction, zeros);
  return differential;
}

//========================================================================
// Equivariant Filter (EqF) Implementation
//========================================================================
/**
 * Initializes the EqF with state dimension validation  and computes lifted
 * innovation mapping
 * @param Sigma Initial covariance
 * @param n Number of calibration states
 * @param m Number of sensors
 * Uses SelfAdjointSolver, completeOrthoganalDecomposition().pseudoInverse()
 */
template <size_t N>
EqF<N>::EqF(const Matrix& Sigma, int m)
    : dof(6 + 3 * N),
      X_hat(G<N>::identity(N)),
      Sigma(Sigma),
      xi_0(State<N>::identity()) {
  if (Sigma.rows() != dof || Sigma.cols() != dof) {
    throw std::invalid_argument(
        "Initial covariance dimensions must match the degrees of freedom");
  }

  // Check positive semi-definite
  Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(Sigma);
  if (eigensolver.eigenvalues().minCoeff() < -1e-10) {
    throw std::invalid_argument(
        "Covariance matrix must be semi-positive definite");
  }

  if (N < 0) {
    throw std::invalid_argument(
        "Number of calibration states must be non-negative");
  }

  if (m <= 1) {
    throw std::invalid_argument(
        "Number of direction sensors must be at least 2");
  }

  // Compute differential of phi
  Dphi0 = stateActionDiff(xi_0);
  InnovationLift = Dphi0.completeOrthogonalDecomposition().pseudoInverse();
}
/**
 * Computes the internal group state to a physical state estimate
 * @return Current state estimate
 */
template <size_t N>
State<N> EqF<N>::stateEstimate() const {
  return X_hat * xi_0;
}
/**
 * Implements the prediction step of the EqF using system dynamics and
 * covariance propagation and advances the filter state by symmtery-preserving
 * dynamics.Uses a Lie group integrator scheme for discrete time propagation
 * @param u Angular velocity measurements
 * @param dt time steps
 * Updated internal state and covariance
 */
template <size_t N>
void EqF<N>::propagation(const Input& u, double dt) {
  State<N> state_est = stateEstimate();
  Vector L = lift(state_est, u);

  Matrix Phi_DT = stateTransitionMatrix(u, dt);
  Matrix Bt = inputMatrixBt();

  Matrix tempSigma = blockDiag(u.Sigma, repBlock(1e-9 * I_3x3, N));
  Matrix M_DT = (Bt * tempSigma * Bt.transpose()) * dt;

  X_hat = X_hat * G<N>::exp(L * dt);
  Sigma = Phi_DT * Sigma * Phi_DT.transpose() + M_DT;
}
/**
 * Implements the correction step of the filter using discrete measurements
 * Computes the measurement residual, Kalman gain and the updates both the state
 * and covariance
 *
 * @param y Measurements
 */
template <size_t N>
void EqF<N>::update(const Measurement& y) {
  if (y.cal_idx > static_cast<int>(N)) {
    throw std::invalid_argument("Calibration index out of range");
  }

  // Get vector representations for checking
  Vector3 y_vec = y.y.unitVector();
  Vector3 d_vec = y.d.unitVector();

  // Skip update if any NaN values are present
  if (std::isnan(y_vec[0]) || std::isnan(y_vec[1]) || std::isnan(y_vec[2]) ||
      std::isnan(d_vec[0]) || std::isnan(d_vec[1]) || std::isnan(d_vec[2])) {
    return;  // Skip this measurement
  }

  Matrix Ct = measurementMatrixC(y.d, y.cal_idx);
  Vector3 action_result = outputAction(X_hat.inv(), y.y, y.cal_idx);
  Vector3 delta_vec = Rot3::Hat(y.d.unitVector()) * action_result;
  Matrix Dt = outputMatrixDt(y.cal_idx);
  Matrix S = Ct * Sigma * Ct.transpose() + Dt * y.Sigma * Dt.transpose();
  Matrix K = Sigma * Ct.transpose() * S.inverse();
  Vector Delta = InnovationLift * K * delta_vec;
  X_hat = G<N>::exp(Delta) * X_hat;
  Sigma = (Matrix::Identity(dof, dof) - K * Ct) * Sigma;
}
/**
 * Computes linearized continuous time state matrix
 * @param u Angular velocity
 * @return Linearized state matrix
 * Uses Matrix zero and Identity functions
 */
template <size_t N>
Matrix EqF<N>::stateMatrixA(const Input& u) const {
  Matrix3 W0 = velocityAction(X_hat.inv(), u).W();
  Matrix A1 = Matrix::Zero(6, 6);
  A1.block<3, 3>(0, 3) = -I_3x3;
  A1.block<3, 3>(3, 3) = W0;
  Matrix A2 = repBlock(W0, N);
  return blockDiag(A1, A2);
}

/**
 * Computes the discrete time state transition matrix
 * @param u Angular velocity
 * @param dt time step
 * @return State transition matrix in discrete time
 */
template <size_t N>
Matrix EqF<N>::stateTransitionMatrix(const Input& u, double dt) const {
  Matrix3 W0 = velocityAction(X_hat.inv(), u).W();
  Matrix Phi1 = Matrix::Zero(6, 6);

  Matrix3 Phi12 = -dt * (I_3x3 + (dt / 2) * W0 + ((dt * dt) / 6) * W0 * W0);
  Matrix3 Phi22 = I_3x3 + dt * W0 + ((dt * dt) / 2) * W0 * W0;

  Phi1.block<3, 3>(0, 0) = I_3x3;
  Phi1.block<3, 3>(0, 3) = Phi12;
  Phi1.block<3, 3>(3, 3) = Phi22;
  Matrix Phi2 = repBlock(Phi22, N);
  return blockDiag(Phi1, Phi2);
}
/**
 * Computes the input uncertainty propagation matrix
 * @return
 * Uses the blockdiag matrix
 */
template <size_t N>
Matrix EqF<N>::inputMatrixBt() const {
  Matrix B1 = blockDiag(X_hat.A.matrix(), X_hat.A.matrix());
  Matrix B2(3 * N, 3 * N);

  for (size_t i = 0; i < N; ++i) {
    B2.block<3, 3>(3 * i, 3 * i) = X_hat.B[i].matrix();
  }

  return blockDiag(B1, B2);
}
/**
 * Computes the linearized measurement matrix. The structure depends on whether
 * the sensor has a calibration state
 * @param d reference direction
 * @param idx Calibration index
 * @return Measurement matrix
 * Uses the matrix zero, Rot3 hat and the Unitvector functions
 */
template <size_t N>
Matrix EqF<N>::measurementMatrixC(const Unit3& d, int idx) const {
  Matrix Cc = Matrix::Zero(3, 3 * N);

  // If the measurement is related to a sensor that has a calibration state
  if (idx >= 0) {
    // Set the correct 3x3 block in Cc
    Cc.block<3, 3>(0, 3 * idx) = Rot3::Hat(d.unitVector());
  }

  Matrix3 wedge_d = Rot3::Hat(d.unitVector());

  // Create the combined matrix
  Matrix temp(3, 6 + 3 * N);
  temp.block<3, 3>(0, 0) = wedge_d;
  temp.block<3, 3>(0, 3) = Matrix3::Zero();
  temp.block(0, 6, 3, 3 * N) = Cc;

  return wedge_d * temp;
}
/**
 * Computes the measurement uncertainty propagation matrix
 * @param idx Calibration index
 * @return Returns B[idx] for calibrated sensors, A for uncalibrated
 */
template <size_t N>
Matrix EqF<N>::outputMatrixDt(int idx) const {
  // If the measurement is related to a sensor that has a calibration state
  if (idx >= 0) {
    if (idx >= static_cast<int>(N)) {
      throw std::out_of_range("Calibration index out of range");
    }
    return X_hat.B[idx].matrix();
  } else {
    return X_hat.A.matrix();
  }
}

}  // namespace abc_eqf_lib

template <size_t N>
struct traits<abc_eqf_lib::EqF<N>>
    : internal::LieGroupTraits<abc_eqf_lib::EqF<N>> {};
}  // namespace gtsam

#endif  // ABC_EQF_H