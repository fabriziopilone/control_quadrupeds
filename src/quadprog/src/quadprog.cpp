#include "quadprog/quadprog.hpp"

#include <vector>

using namespace Eigen;

int min(int a, int b)
{
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

int solve_quadprog(
    MatrixXd& G,
    VectorXd& g0,
    MatrixXd& C,
    VectorXd& c0,
    VectorXd& x,
    int meq,
    int factorized
) {
    int n = G.rows();
    int q = C.cols();

    double obj;

    std::vector<double> lagr(q);
    std::vector<int> iact(q);
    int nact;
    std::vector<int> iter(2);
    std::vector<double> work(2*n + 2*q + min(n,q)*(min(n,q)+5)/2);

    return qpgen2_(
        G.data(), g0.data(), n,
        x.data(), &lagr[0], &obj,
        C.data(), c0.data(), q, meq,
        &iact[0], &nact, &iter[0],
        &work[0], factorized
    );
}