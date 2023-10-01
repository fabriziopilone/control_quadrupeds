#pragma once

#include "quadprog/solve.QP.hpp"

#include <Eigen/Core>

/// @brief Solves the following problem:
// Minimize     1/2 x^T G x - g0^T x
// Subject to   CE.T x - ce0  = 0
//              CI.T x - ci0 >= 0
//
// Where CE = C[:, :m_eq]
//       CI = C[:, m_eq:]
//       ce0 = c0[:meq]
//       ci0 = c0[meq:]
///
/// @param[in]  G
/// @param[in]  g0
/// @param[in]  C
/// @param[in]  c0
/// @param[out] x
/// @param[in]  meq
/// @param[in]  factorized
///
/// @retval 0 OK
/// @retval 1 Constraints are inconsistent, no solution.
/// @retval 2 Matrix G is not positive definite.
int solve_quadprog(
    Eigen::MatrixXd& G,
    Eigen::VectorXd& g0,
    Eigen::MatrixXd& C,
    Eigen::VectorXd& c0,
    Eigen::VectorXd& x,
    int meq=0,
    int factorized=0
);

/// @brief Solves the following problem:
// Minimize     1/2 x^T G x - g0^T x
// Subject to   CE.T x - ce0  = 0
//              CI.T x - ci0 >= 0
//
// Where CE = C[:, :m_eq]
//       CI = C[:, m_eq:]
//       ce0 = c0[:meq]
//       ci0 = c0[meq:]
///
/// @param[in]  G
/// @param[in]  g0
/// @param[in]  C
/// @param[in]  c0
/// @param[out] x
/// @param[in]  meq
/// @param[in]  factorized
///
/// @retval 0 OK
/// @retval 1 Constraints are inconsistent, no solution.
/// @retval 2 Matrix G is not positive definite.
inline int solve_quadprog(
    Eigen::MatrixXd&& G,
    Eigen::VectorXd&& g0,
    Eigen::MatrixXd&& C,
    Eigen::VectorXd&& c0,
    Eigen::VectorXd& x,
    int meq=0,
    int factorized=0
) {
    return solve_quadprog(
        G,
        g0,
        C,
        c0,
        x,
        meq,
        factorized
    );
}
