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

#include "quadprog/quadprog.hpp"
#include "hierarchical_optimization/HO.hpp"
#include <iostream>
using namespace task;
using namespace Eigen;

HO::HO(){
    this->size = 0;
}
HO::HO(std::vector<task::Task> task_vec, int size){
    this->task_vec = task_vec;
    this->size = size;
};

Eigen::VectorXd HO::solve_ho(std::vector<std::string> task_names){

    if (size == 0)
        throw std::invalid_argument("The task vector must not be empty");

    for (int i=0; i<=size-1; i++){
        if(typeid(task_vec[i]) != typeid(task::Task))
            throw std::invalid_argument("The task vector must contain only Task type");
    }
    
    task::Task temp;
    int rows_A = task_vec[0].get_A().rows();
    int cols_A = task_vec[0].get_A().cols();
    int rows_D = task_vec[0].get_D().rows();

    MatrixXd Z_p = MatrixXd::Identity(cols_A, cols_A);
    VectorXd xi_opt = VectorXd::Zero(cols_A + rows_D);
    VectorXd x_opt = VectorXd::Zero(cols_A);
    VectorXd v_opt_vec = VectorXd::Zero(rows_D);
    VectorXd z_opt = VectorXd::Zero(cols_A);

    MatrixXd H = MatrixXd::Zero(cols_A+rows_D, cols_A+rows_D);
    VectorXd c = VectorXd::Zero(cols_A + rows_D);
    MatrixXd D_all = task_vec[0].get_D();
    MatrixXd D_hat = MatrixXd::Zero(rows_D+rows_D, cols_A+rows_D);

    VectorXd f_all(rows_D);
    VectorXd f_hat = VectorXd::Zero(rows_D+rows_D);
    int sol = 0;

    /*
    Eigen::MatrixXd A_p1 = MatrixXd::Zero(rows_A, cols_A);xd
    Eigen::MatrixXd D_p1 = MatrixXd::Zero(rows_D, cols_A);
    Eigen::VectorXd b_p1 = VectorXd::Zero(rows_A);
    Eigen::VectorXd f_p1 = VectorXd::Zero(rows_D);
    */

    for (int i=0; i<=size-1; i++){
        std::cout <<"\n\n******************** Solving task " <<task_names[i] <<" ******************** \n\n";
        //std::cout <<"Proiettore nel nullo Zp:\n " <<Z_p <<"\n";
        //std::cout <<"Dimensione proiettore nel nullo:\n" <<Z_p.rows() << "  " <<Z_p.cols() <<"\n";
        temp.set_task(task_vec[i].get_A(), task_vec[i].get_b(), task_vec[i].get_D(), task_vec[i].get_f());
        MatrixXd A_p1 = temp.get_A();
        MatrixXd D_p1 = temp.get_D();
        VectorXd b_p1 = temp.get_b();
        VectorXd f_p1 = temp.get_f();
        rows_D = D_p1.rows();

        //std::cout <<"Dimensione A:\n" <<A_p1.rows() << "  " <<A_p1.cols() <<"\n";
        //std::cout <<"Dimensione b:\n" <<b_p1.rows() << "  " <<b_p1.cols() <<"\n";
        //std::cout <<"Dimensione D:\n" <<D_p1.rows() << "  " <<D_p1.cols() <<"\n";
        //std::cout <<"Dimensione f:\n" <<f_p1.rows() << "  " <<f_p1.cols() <<"\n";

        H.conservativeResize(cols_A+rows_D, cols_A+rows_D);
        c.conservativeResize(cols_A+rows_D);
        f_hat.conservativeResize(rows_D+rows_D);

        H.setZero();
        c.setZero();
        f_hat.setZero();

        // H MATRIX
        H.topLeftCorner(cols_A, cols_A) = Z_p.transpose()*A_p1.transpose()*A_p1*Z_p + temp.get_reg()*MatrixXd::Identity(cols_A, cols_A);
        H.bottomLeftCorner(rows_D, cols_A) = MatrixXd::Zero(rows_D, cols_A);
        H.topRightCorner(cols_A, rows_D) = MatrixXd::Zero(cols_A, rows_D);
        H.bottomRightCorner(rows_D, rows_D) = MatrixXd::Identity(rows_D, rows_D);
        //std::cout << "Matrice H: " <<H <<"\n";
        //std::cout <<"Regolarizzazione: " << temp.get_reg() <<"\n";

        // c VECTOR
        c.head(cols_A) = Z_p.transpose()*A_p1.transpose()*(A_p1*x_opt - b_p1);
        c.tail(rows_D) = VectorXd::Zero(rows_D);
        //std::cout << "Matrice c: " <<c <<"\n";

        // D MATRIX
        if (i==0){
            D_hat.bottomLeftCorner(rows_D, cols_A) = MatrixXd::Zero(rows_D, cols_A);
            D_hat.bottomRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
            D_hat.topLeftCorner(rows_D, cols_A) = D_p1;
            D_hat.topRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
        }
        else{
            int rows_Dall = D_all.rows();
            MatrixXd temp_D = D_all;
            D_all.conservativeResize(rows_Dall+rows_D, NoChange);
            D_all.topRows(rows_D) = D_p1;
            D_all.bottomRows(rows_Dall) = temp_D;

            D_hat.conservativeResize(D_all.rows()+rows_D, cols_A+rows_D);
            D_hat.setZero();
            D_hat.topLeftCorner(D_all.rows(), cols_A) = D_all*Z_p;
            D_hat.bottomLeftCorner(rows_D, cols_A) = MatrixXd::Zero(rows_D, cols_A);
            /*
            MatrixXd temporanea(D_all.rows(), rows_D);
            temporanea << -MatrixXd::Identity(rows_D, rows_D), MatrixXd::Zero(D_all.rows()-rows_D, rows_D);
            */
            D_hat.topRightCorner(D_all.rows(), rows_D) << -MatrixXd::Identity(rows_D, rows_D), MatrixXd::Zero(D_all.rows()-rows_D, rows_D);
            D_hat.bottomRightCorner(rows_D, rows_D) = -MatrixXd::Identity(rows_D, rows_D);
            //std::cout << "Matrice D_hat: " <<D_hat <<"\n";
        }   

        // f VECTOR

        if (i==0)
            f_all = f_p1;
        else{
            int rows_fall = f_all.rows();
            VectorXd temp_f = f_all;
            
            f_all.conservativeResize(rows_fall+rows_D, NoChange);
            f_all.setZero();
            f_all.head(rows_D) = f_p1;
            f_all.tail(rows_fall) = temp_f; 
        }

        f_hat = f_all - D_all*x_opt;

        if (i != 0){
            VectorXd v_ext = VectorXd::Zero(v_opt_vec.rows()+rows_D);
            v_ext.tail(v_opt_vec.rows()) = v_opt_vec;
            f_hat = f_hat.eval() + v_ext;
        }

        f_hat.conservativeResize(f_all.rows()+rows_D);
        f_hat.tail(rows_D) = VectorXd::Zero(rows_D);
        //std::cout << "Matrice f_hat: " <<f_hat <<"\n";

        /*
        SOLVING QP PROBLEM
        */
        xi_opt.conservativeResize(cols_A+rows_D);
        xi_opt.setZero();

        c = -c.eval();
        D_hat.transposeInPlace();
        D_hat = -D_hat.eval();
        f_hat = -f_hat.eval();
        sol = solve_quadprog(
        H, 
        c, 
        D_hat, 
        f_hat, 
        xi_opt,
        0);

       //std::cout <<"Prova di stampa dopo chiamata a solve_quadprog\n";
        if (sol == 1) {
        std::cerr << "At priority " << i<< ", corresponding to task " <<task_names[i] <<", constraints are inconsistent, no solution." << '\n' << std::endl;
        } else if (sol == 2) {
        std::cerr << "At priority " << i << ", matrix G is not positive definite." << '\n' << std::endl;
         }

        //std::cout <<"Dimensione vettore soluzione: \n" <<xi_opt.rows() <<"\n";
        //std::cout << "Vettore soluzione xi_opt: " <<xi_opt <<"\n";

       //z_opt = xi_opt(seq(0, cols_A-1));
       z_opt = xi_opt.segment(0, cols_A);
       //std::cout <<"z_opt, of dimension " <<z_opt.rows()<<" at step " << i<< " :\n" <<z_opt <<"\n";
       Eigen::VectorXd x_temp = x_opt;
       x_opt = x_temp.eval() + Z_p*z_opt;
       //std::cout << "x_opt at step " << i <<" :\n" << x_opt <<"\n";

       VectorXd v_temp = v_opt_vec;
       if(i != 0){
            v_opt_vec.resize(v_opt_vec.rows()+rows_D);
            v_opt_vec.head(rows_D) = xi_opt.tail(rows_D);
            v_opt_vec.tail(v_temp.rows()) = v_temp;
       }
       else{v_opt_vec = xi_opt.tail(rows_D);}
       //std::cout <<"v_opt_vec at step " << i << " :\n" <<v_opt_vec <<"\n";

       // UPDATING NULL SPACE PROJECTOR
       /*
       MatrixXd Null_AZp(rows_A, rows_A);
       MatrixXd M = A_p1*Z_p;
       Eigen::MatrixXd pInv = M.completeOrthogonalDecomposition().pseudoInverse();
       Null_AZp = MatrixXd::Identity(rows_A, rows_A) - pInv*M;
       Z_p = Z_p*Null_AZp;
       */
       Z_p = Z_p.eval() * null_space_projector(A_p1*Z_p.eval());
       //std::cout <<"Zp: " <<Z_p <<"\n";
    }
    //std::cout <<"Stampa x_opt dopo il for:\n" <<x_opt <<"\n" <<std::endl;
    return x_opt;
};

template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoinverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
    MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
    0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
    tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

/* ========================== null_space_projector ========================== */

inline MatrixXd HO::null_space_projector(const MatrixXd& M)
{
    // return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
    //        - pseudoinverse(M) * M;

    return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
           - M.completeOrthogonalDecomposition().pseudoInverse().eval() * M;
}