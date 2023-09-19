#include "quadprog/quadprog.hpp"

#include <iostream>

int main()
{
    using namespace Eigen;

    MatrixXd G = MatrixXd::Identity(2,2);
    VectorXd g0 = VectorXd::Ones(2);

    MatrixXd CI = MatrixXd::Identity(2,2);
    VectorXd ci0 = 2 * VectorXd::Ones(2);

    VectorXd x = VectorXd::Zero(2);

    solve_quadprog(
        G,
        g0,
        CI,
        ci0,
        x,
        0,
        0
    );

    std::cout << x << std::endl;

    return 0;
}