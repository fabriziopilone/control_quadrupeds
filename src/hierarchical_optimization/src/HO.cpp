/* 
A task T is a set of linear equality and/or inequality constraints on x:
    A*x - b = w
T:{
    D*x - f <= v
with w, v slack variables to be minimized

If we have more tasks T1, T2, ..., Tn to be solved in a prioritized order, we need to find the solution of the 
(k+1)-th tasks, x_(k+1), in the null space of all higher priority equality constraints: Zp = Null(A_bar_p)
with A_bar_p = [A1' ...  Ap']'
from which x = x* + Zp * z_(p+1), with z_(p+1) a vector living the row space of Zp.

Solving for a new task T_(p+1) means computing z*_(p+1) and v*_(p+1) from the QP problem:
min_( z_(p+1), v_(p+1) ) 0.5*|| A_(p+1)*(x* + Zp*z_(p+1) - b_(p+1) ||^2 + 0.5*|| v_(p+1)||^2
subject to
D_(p+1)*(x* + Zp*z_(p+1) ) - f_(p+1) <= v_(p+1)
D_p*(x* +Zp*z_(p+1) ) - f_p <= v*_p
...
D_1*(x* + Zp*z_(p+1) ) - f_1 <= v*_1
v_(p+1) >= 0

Writing xi_p = [z_p; v_p] we can write
min_xi_(p+1) 0.5*xi_(p+1)' * H_(p+1) * xi_(p+1) + c_(p+1)' * xi_(p+1)
subject to
D_(p+1)^hat * xi_(p+1) <= f_(p+1)^hat
with
H_(p+1) = [Zp' * A_(p+1)' * A_(p+1) * Zp,     0;]
          [               0,                  I ]

c_(p+1) = [Zp' * A_(p+1)' * ( A_(p+1) * x* - b_(p+1) );]
          [               0                            ]

D_(p+1)^hat = [D_(p+1)*Zp, -eye();]
              [D_p*Zp,         0; ]
              [          ...      ]
              [D_1*Zp,         0; ]
              [   0,          -I  ]

f_(p+1)^hat = [ f_(p+1) - D_(p+1)*x*; ]
              [ f_p - D_p*x* + v_(p)*;]
              [           ...         ]
              [f_1 - D_1*x* + v_(1)*; ]
              [            0          ]

Iteratively
Z_2 = Null(A_1)*Null( A_2*Null(A_1) ) = Z_1 * Null( A_2 * Z_1 )

*/

#include "Task.cpp"
#include "quadprog/quadprog.hpp"
using namespace task;
using namespace Eigen;

Eigen::VectorXd solve_ho(Task task_vec[], int size){

    if (size == 0)
        throw std::invalid_argument("The task vector must not be empty");

    for (int i=0; i<size; i++){
        if(typeid(task_vec[i]) != typeid(Task))
            throw std::invalid_argument("The task vector must contain only Task type");
    }
    
    Task temp;
    int rows_A = task_vec[0].get_A().rows();
    int rows_D = task_vec[0].get_D().rows();
    MatrixXd Z_p = MatrixXd::Identity(rows_A, rows_A);
    VectorXd xi_opt = VectorXd::Zero(rows_A + rows_D);
    VectorXd x_opt = VectorXd::Zero(rows_A);
    VectorXd v_opt_vec = VectorXd::Zero(rows_D);
    VectorXd z_opt = VectorXd::Zero(rows_A);

    MatrixXd H(rows_A+rows_D, rows_A+rows_D);
    VectorXd c(rows_A, rows_D);
    MatrixXd D_all = task_vec[0].get_D();
    MatrixXd D_hat(rows_D+rows_D, rows_D+rows_D);

    VectorXd f_all(rows_D);
    VectorXd f_hat(rows_D+rows_D);

    for (int i=0; i<size; i++){
        temp = task_vec[i];
        MatrixXd A_p1 = temp.get_A();
        MatrixXd D_p1 = temp.get_D();
        VectorXd b_p1 = temp.get_b();
        VectorXd f_p1 = temp.get_f();
        rows_D = D_p1.rows();

        // H MATRIX
        H.topLeftCorner(rows_A, rows_A) = Z_p.transpose()*A_p1.transpose()*A_p1*Z_p;
        H.bottomLeftCorner(rows_D, rows_A) = MatrixXd::Zero(rows_D, rows_A);
        H.topRightCorner(rows_A, rows_D) = MatrixXd::Zero(rows_A, rows_D);
        H.bottomRightCorner(rows_D, rows_D) = MatrixXd::Identity(rows_D, rows_D);

        // c VECTOR
        c.head(rows_A) = Z_p.transpose()*A_p1.transpose()*(A_p1*x_opt - b_p1);
        c.tail(rows_D) = VectorXd::Zero(rows_D);

        // D MATRIX
        if (i==0){
            D_hat.bottomLeftCorner(rows_D, rows_A) = MatrixXd::Zero(rows_D, rows_A);
            D_hat.bottomRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
            D_hat.topLeftCorner(rows_D, rows_A) = D_p1;
            D_hat.topRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
        }
        else{
            int rows_Dall = D_all.rows();
            MatrixXd temp_D = D_all;
            MatrixXd Z_p_vec(i*rows_A, rows_A);
            for (int j=0; j<=i; j++){
                Z_p_vec = Z_p, Z_p;
            }
            
            D_all.conservativeResize(rows_Dall+rows_D, NoChange);
            D_all.topRows(rows_D) = D_p1;
            D_all.bottomRows(rows_Dall) = temp_D;

            D_hat.resize(D_all.rows()+rows_D, rows_A+rows_D);
            D_hat.topLeftCorner(D_all.rows(), rows_A) = D_all*Z_p_vec;
            D_hat.bottomLeftCorner(rows_D, rows_A) = MatrixXd::Zero(rows_D, rows_A);

            D_hat.topRightCorner(D_all.rows(), rows_A) = -MatrixXd::Identity(rows_D, rows_D), MatrixXd::Zero(D_all.rows()-rows_D, rows_D);
            D_hat.bottomRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
        }   

        // f VECTOR

        if (i==0)
            f_all = f_p1;
        else{
            int rows_fall = f_all.rows();
            VectorXd temp_f = f_all;
            
            f_all.resize(rows_fall+rows_D, NoChange);
            f_all.head(rows_D) = f_p1;
            f_all.tail(rows_fall) = temp_f; 
        }

        f_hat.resize(f_all.rows());
        VectorXd x_opt_vec(D_all.rows());
        //for (int j=0; j<=i; j++)
        //    x_opt_vec = x_opt, x_opt;

        f_hat = f_all - D_all*x_opt;

        if (i != 0){
            VectorXd v_ext(v_opt_vec.rows()+rows_D);
            v_ext = VectorXd::Zero(rows_D), v_opt_vec;
            f_hat = f_hat + v_ext;
        }

        f_hat.conservativeResize(f_all.rows()+rows_D);
        f_hat.tail(rows_D) = VectorXd::Zero(rows_D);

        /*
        SOLVING QP PROBLEM
        */
       D_hat = -D_hat;
       const int sol = solve_quadprog(H, c, D_hat, f_hat, xi_opt);

       z_opt = xi_opt(seq(0, rows_A));
       x_opt = x_opt + Z_p*z_opt;

       VectorXd v_temp = v_opt_vec;
       v_opt_vec.resize(v_opt_vec.rows()+rows_D);
       v_opt_vec.head(rows_D) = xi_opt.tail(v_temp.rows());
       v_opt_vec.tail(v_temp.rows()) = v_temp;

       // UPDATING NULL SPACE PROJECTOR
       MatrixXd Null_AZp(rows_A, rows_A);
       MatrixXd M = A_p1*Z_p;
       Eigen::MatrixXd pInv = M.completeOrthogonalDecomposition().pseudoInverse();
       Null_AZp = MatrixXd::Identity(rows_A, rows_A) - pInv*M;
       Z_p = Z_p*Null_AZp;

    }
};