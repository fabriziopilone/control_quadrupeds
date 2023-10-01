#include "hierarchical_optimization/Task.hpp"
#include <Eigen/Dense>
#include "quadprog/quadprog.hpp"
#include <iostream>

using namespace Eigen;

/* 
A task T is a set of linear equality and/or inequality constraints on x:
    A*x - b = w
T:{
    D*x - f <= v
with w, v slack variables to be minimized
x in R^n, b in R^n, A in R^(n*n), w in R^n, D in R^(m*n), f in R^m, v in R^m



Solving for a task T means computing z* and v* from the QP problem:
min_( z, v) ) 0.5*|| A*z - b ||^2 + 0.5*|| v||^2
subject to
D*z - f <= v
v >= 0

Writing xi_p = [z; v] we can write
min_xi 0.5*xi' * H * xi + c' * xi
subject to
D^hat * xi <= f^hat
with
H = [ A' * A,     0;]
    [    0,       I ]

c = [ -A' * b;]
    [    0    ]

D^hat = [D,  -I; ]
        [0,  -I  ]

f^hat = [f]
        [0]

xi in R^(n+m), H in R^([n+m]*[n+m]), c in R^(n+m), D_hat in R^([m+m]*[n+m]), f_hat in R^(m+m)

*/

namespace task{

Task::Task(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd D, Eigen::VectorXd f){
    if (b.size() != A.rows())
        throw std::invalid_argument("Il numero di righe di b deve essere uguale al numero di righe di A");
    if (f.size() != D.rows())
        throw std::invalid_argument("Il numero di righe di f deve essere uguale al numero di righe di D");
    if (D.cols() != A.cols())
        throw std::invalid_argument("Il numero di colonne di D deve essere uguale al numero di colonne di A");
    this->A = A;
    this->b = b;
    this->D = D;
    this->f = f;
}

Task::Task(){
    this->A = MatrixXd::Zero(1,1);
    this->b = VectorXd::Zero(1);
    this->D = MatrixXd::Zero(1,1);
    this->f = VectorXd::Zero(1);
}

Eigen::VectorXd Task::solve_QP(){
    int col_A = this->A.cols();
    int row_D = this->D.rows();

    std::cout <<"this->A: \n" << this->A << "\n";
    std::cout <<"this->b: \n" << this->b << "\n";
    std::cout <<"this->D: \n" << this->D << "\n";
    std::cout <<"this->f: \n" << this->f << "\n";

    /*
    H = [A'A 0]
        [ 0  I]
    */
   Eigen::MatrixXd H(col_A + row_D, col_A+row_D);
   H.topLeftCorner(col_A, col_A) = A.transpose()*A + this->reg*MatrixXd::Identity(col_A, col_A);
   H.topRightCorner(col_A, row_D) = MatrixXd::Zero(col_A, row_D);
   H.bottomLeftCorner(row_D, col_A) = MatrixXd::Zero(row_D, col_A);
   H.bottomRightCorner(row_D, row_D) = MatrixXd::Identity(row_D, row_D);
   std::cout << "H: " << H <<"\n";

    /*
    c^hat = [-A'b]
            [  0 ]
    */
   Eigen::VectorXd c_hat(col_A+row_D);
   c_hat.head(col_A) = -this->A.transpose()*this->b;
   c_hat.tail(row_D) = VectorXd::Zero(row_D);
   std::cout << "c_hat: " <<c_hat <<"\n";

   /*
   D^hat = [D, -I;]
           [0, -I ]
   */
  Eigen::MatrixXd D_hat(row_D+row_D, col_A+row_D);
  D_hat.topLeftCorner(row_D, col_A) = this->D;
  D_hat.topRightCorner(row_D, row_D) = -MatrixXd::Identity(row_D, row_D);
  //D_hat.topRightCorner(row_D, row_D) = MatrixXd::Zero(row_D, row_D);
  D_hat.bottomLeftCorner(row_D, col_A) = MatrixXd::Zero(row_D, col_A);
  D_hat.bottomRightCorner(row_D, row_D) = -MatrixXd::Identity(row_D, row_D);
  std::cout <<"D_hat: " << D_hat << "\n";

  /*
  f^hat = [f]
          [0]
  */
 Eigen::VectorXd f_hat(row_D+row_D);
  f_hat.head(row_D) = this->f;
  f_hat.tail(row_D) = VectorXd::Zero(row_D);
 std::cout <<"f_hat: " << f_hat << "\n";

// SOLVING QP PROBLEM WITH  EIGEN-QP

/*
    From quadprog documentation:

    Solve a strictly convex quadratic program

    Minimize     1/2 x^T G x - g0^T x
    Subject to   CE.T x - ce0  = 0
                 CI.T x - ci0 >= 0

    Where CE = C[:, :m_eq]
      CI = C[:, m_eq:]
      ce0 = c0[:meq]
      ci0 = c0[meq:]

    int result = solve_quadprog(
        G,
        g0,
        C,
        c0,
        x,
        m_eq [default = 0],
        factorized [default = 0]
    );

    To solve the problem written in former form we must put
*/
    Eigen::VectorXd x_opt(col_A+row_D);
    c_hat = -c_hat.eval();
    D_hat.transposeInPlace(); // solver solves for >=0
    D_hat = -D_hat.eval();
    //f_hat = -f_hat.eval();
    const int sol = solve_quadprog(H, c_hat, D_hat, f_hat, x_opt);
    if (sol == 1)
        throw std::invalid_argument("Non feasible problem");
    return x_opt;
}
}