//#include "hierarchical_optimization/Task.hpp"
# include "/home/fabrizio/Github/control_quadrupeds/hierarchical_optimization/hierarchical_optimization/include/hierarchical_optimization/Task.hpp"
#include <Eigen/Dense>
#include "eigen-qp.hpp"

using namespace Eigen;
using namespace EigenQP;

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

Task::Task(Eigen::MatrixXd A, Eigen::VectorXd, Eigen::MatrixXd D, Eigen::VectorXd f){
    this->A = A;
    this->b = b;
    this->D = D;
    this->f = f;
}

Task::Task(){
    this->A = MatrixXd::Zero();
    this->b = VectorXd::Zero();
    this->D = MatrixXd::Zero();
    this->f = VectorXd::Zero();
}

Eigen::VectorXd Task::solve_QP(){
    int row_A = this->A.rows();
    int row_D = this->D.rows();

    /*
    H = [A'A 0]
        [ 0  I]
    */
   Eigen::MatrixXd H(row_A + row_D);
   H.topLeftCorner(row_A, row_A) = A.transpose()*A;
   H.topRightCorner(row_A, row_D) = MatrixXd::Zero(row_A, row_D);
   H.bottomLeftCorner(row_D, row_A) = MatrixXd::Zero(row_D, row_A);
   H.bottomRightCorner(row_D, row_D) = MatrixXd::Identity(row_D, row_D);

    /*
    c^hat = [-A'b]
            [  0 ]
    */
   Eigen::VectorXd c_hat(row_A+row_D);
   c_hat.head(row_A) = -this->A.transpose()*this->b;
   c_hat.tail(row_D) = VectorXd::Zero(row_D);

   /*
   D^hat = [D, -I;]
           [0, -I ]
   */
  Eigen::MatrixXd D_hat(row_D+row_D, row_A+row_D);
  D_hat.topLeftCorner(row_D, row_A) = this->D;
  D_hat.topRightCorner(row_D, row_D) = -MatrixXd::Identity(row_D, row_D);
  D_hat.bottomLeftCorner(row_D, row_A) = MatrixXd::Zero(row_D, row_A);
  D_hat.bottomRightCorner(row_D, row_D) = -MatrixXd::Identity(row_D, row_D);

  /*
  f^hat = [f]
          [0]
  */
 Eigen::VectorXd f_hat(row_D);
 f_hat = this->f, VectorXd::Zero(row_D);

// SOLVING QP PROBLEM WITH  EIGEN-QP
    Eigen::VectorXd x_opt(row_A);
    quadprog(A, b, D, f, x_opt);
}
}