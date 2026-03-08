/**
 * @file ABC.h
 * @brief Core components for Attitude-Bias-Calibration systems
 *
 * This file contains fundamental components and utilities for the ABC system
 * based on the paper "Overcoming Bias: Equivariant Filter Design for Biased
 * Attitude Estimation with Online Calibration" by Fornasier et al.
 * Authors: Darshan Rajasekaran & Jennifer Oum
 */
#ifndef ABC_H
#define ABC_H
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

namespace gtsam {
namespace abc_eqf_lib {
using namespace std;
using namespace gtsam;
//========================================================================
// Utility Functions
//========================================================================

//========================================================================
// Utility Functions
//========================================================================

/// Check if a vector is a unit vector

bool checkNorm(const Vector3& x, double tol = 1e-3);

/// Check if vector contains NaN values

bool hasNaN(const Vector3& vec);

/// Create a block diagonal matrix from two matrices

Matrix blockDiag(const Matrix& A, const Matrix& B);

/// Repeat a block matrix n times along the diagonal

Matrix repBlock(const Matrix& A, int n);

// Utility Functions Implementation

/**
 * @brief Verifies if a vector has unit norm within tolerance
 * @param x 3d vector
 * @param tol optional tolerance
 * @return Bool indicating that the vector norm is approximately 1
 */
bool checkNorm(const Vector3& x, double tol) {
  return abs(x.norm() - 1) < tol || std::isnan(x.norm());
}

/**
 * @brief Checks if the input vector has any NaNs
 * @param vec A 3-D vector
 * @return true if present, false otherwise
 */
bool hasNaN(const Vector3& vec) {
  return std::isnan(vec[0]) || std::isnan(vec[1]) || std::isnan(vec[2]);
}

/**
 * @brief Creates a block diagonal matrix from input matrices
 * @param A Matrix A
 * @param B Matrix B
 * @return A single consolidated matrix with A in the top left and B in the
 * bottom right
 */
Matrix blockDiag(const Matrix& A, const Matrix& B) {
  if (A.size() == 0) {
    return B;
  } else if (B.size() == 0) {
    return A;
  } else {
    Matrix result(A.rows() + B.rows(), A.cols() + B.cols());
    result.setZero();
    result.block(0, 0, A.rows(), A.cols()) = A;
    result.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
    return result;
  }
}

/**
 * @brief Creates a block diagonal matrix by repeating a matrix 'n' times
 * @param A A matrix
 * @param n Number of times to be repeated
 * @return Block diag matrix with A repeated 'n' times
 */
Matrix repBlock(const Matrix& A, int n) {
  if (n <= 0) return Matrix();

  Matrix result = A;
  for (int i = 1; i < n; i++) {
    result = blockDiag(result, A);
  }
  return result;
}

//========================================================================
// Core Data Types
//========================================================================

/// Input struct for the Biased Attitude System

struct Input {
  Vector3 w;              /// Angular velocity (3-vector)
  Matrix Sigma;           /// Noise covariance (6x6 matrix)
  static Input random();  /// Random input
  Matrix3 W() const {     /// Return w as a skew symmetric matrix
    return Rot3::Hat(w);
  }
};

/// Measurement struct
struct Measurement {
  Unit3 y;           /// Measurement direction in sensor frame
  Unit3 d;           /// Known direction in global frame
  Matrix3 Sigma;     /// Covariance matrix of the measurement
  int cal_idx = -1;  /// Calibration index (-1 for calibrated sensor)
};

/// State class representing the state of the Biased Attitude System
template <size_t N>
class State {
 public:
  Rot3 R;                 // Attitude rotation matrix R
  Vector3 b;              // Gyroscope bias b
  std::array<Rot3, N> S;  // Sensor calibrations S

  /// Constructor
  State(const Rot3& R = Rot3::Identity(), const Vector3& b = Vector3::Zero(),
        const std::array<Rot3, N>& S = std::array<Rot3, N>{})
      : R(R), b(b), S(S) {}

  /// Identity function
  static State identity() {
    std::array<Rot3, N> S_id{};
    S_id.fill(Rot3::Identity());
    return State(Rot3::Identity(), Vector3::Zero(), S_id);
  }
  /**
   * Compute Local coordinates in the state relative to another state.
   * @param other The other state
   * @return Local coordinates in the tangent space
   */
  Vector localCoordinates(const State<N>& other) const {
    Vector eps(6 + 3 * N);

    // First 3 elements - attitude
    eps.head<3>() = Rot3::Logmap(R.between(other.R));
    // Next 3 elements - bias
    // Next 3 elements - bias
    eps.segment<3>(3) = other.b - b;

    // Remaining elements - calibrations
    for (size_t i = 0; i < N; i++) {
      eps.segment<3>(6 + 3 * i) = Rot3::Logmap(S[i].between(other.S[i]));
    }

    return eps;
  }

  /**
   * Retract from tangent space back to the manifold
   * @param v Vector in the tangent space
   * @return New state
   */
  State retract(const Vector& v) const {
    if (v.size() != static_cast<Eigen::Index>(6 + 3 * N)) {
      throw std::invalid_argument(
          "Vector size does not match state dimensions");
    }
    Rot3 newR = R * Rot3::Expmap(v.head<3>());
    Vector3 newB = b + v.segment<3>(3);
    std::array<Rot3, N> newS;
    for (size_t i = 0; i < N; i++) {
      newS[i] = S[i] * Rot3::Expmap(v.segment<3>(6 + 3 * i));
    }
    return State(newR, newB, newS);
  }
};

//========================================================================
// Symmetry Group
//========================================================================

/**
 * Symmetry group (SO(3) |x so(3)) x SO(3) x ... x SO(3)
 * Each element of the B list is associated with a calibration state
 */
template <size_t N>
struct G {
  Rot3 A;                 /// First SO(3) element
  Matrix3 a;              /// so(3) element (skew-symmetric matrix)
  std::array<Rot3, N> B;  /// List of SO(3) elements for calibration

  /// Initialize the symmetry group G
  G(const Rot3& A = Rot3::Identity(), const Matrix3& a = Matrix3::Zero(),
    const std::array<Rot3, N>& B = std::array<Rot3, N>{})
      : A(A), a(a), B(B) {}

  /// Group multiplication
  G operator*(const G<N>& other) const {
    std::array<Rot3, N> newB;
    for (size_t i = 0; i < N; i++) {
      newB[i] = B[i] * other.B[i];
    }
    return G(A * other.A, a + Rot3::Hat(A.matrix() * Rot3::Vee(other.a)), newB);
  }

  /// Group inverse
  G inv() const {
    Matrix3 Ainv = A.inverse().matrix();
    std::array<Rot3, N> Binv;
    for (size_t i = 0; i < N; i++) {
      Binv[i] = B[i].inverse();
    }
    return G(A.inverse(), -Rot3::Hat(Ainv * Rot3::Vee(a)), Binv);
  }

  /// Identity element
  static G identity(int n) {
    std::array<Rot3, N> B;
    B.fill(Rot3::Identity());
    return G(Rot3::Identity(), Matrix3::Zero(), B);
  }

  /// Exponential map of the tangent space elements to the group
  static G exp(const Vector& x) {
    if (x.size() != static_cast<Eigen::Index>(6 + 3 * N)) {
      throw std::invalid_argument("Vector size mismatch for group exponential");
    }
    Rot3 A = Rot3::Expmap(x.head<3>());
    Vector3 a_vee = Rot3::ExpmapDerivative(-x.head<3>()) * x.segment<3>(3);
    Matrix3 a = Rot3::Hat(a_vee);
    std::array<Rot3, N> B;
    for (size_t i = 0; i < N; i++) {
      B[i] = Rot3::Expmap(x.segment<3>(6 + 3 * i));
    }
    return G(A, a, B);
  }
};
}  // namespace abc_eqf_lib

template <size_t N>
struct traits<abc_eqf_lib::State<N>>
    : internal::LieGroupTraits<abc_eqf_lib::State<N>> {};

template <size_t N>
struct traits<abc_eqf_lib::G<N>> : internal::LieGroupTraits<abc_eqf_lib::G<N>> {
};

}  // namespace gtsam

#endif  // ABC_H
