#include "hierarchical_optimization/Task.hpp"
#include "hierarchical_optimization/HO.hpp"
#include <stdio.h>
#include <iostream>
using namespace task;
using namespace Eigen;

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

inline MatrixXd null_space_projector(const MatrixXd& M)
{
    // return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
    //        - pseudoinverse(M) * M;

    return   Eigen::MatrixXd::Identity(M.cols(), M.cols())
           - M.completeOrthogonalDecomposition().pseudoInverse().eval() * M;
}

int main(){

    /*
    A*x - b = w
    D*x  - f <= v

    [1 ]*x - [-20 ] <= v    => x+20 <=v -> x <= v-20
    [-1]     [-100]         => -x+100 <= v -> x >= 100-v
    */
    Eigen::MatrixXd A1(1,2);
    Eigen::VectorXd b1(1);
    Eigen::MatrixXd D1(2,2);
    Eigen::VectorXd f1(2);

    A1 << 1, 0;
    b1 << 0;
    D1 << 1, 0, 
          1, 0;
    f1 << -20,
          -20;
    /*
    minimze x^2 + v^2
    s.t     x + 20 <= v
    */

    Task test1(A1, b1, D1, f1);
    std::cout <<test1 << "\n";
    Eigen::VectorXd x_opt(3);
    x_opt = test1.solve_QP();
    std::cout << "Soluzione task 1 singolo: " << x_opt <<"\n";

    std::cout << "Prova proiezione nullo \n" << null_space_projector(A1) <<"\n";


    Eigen::MatrixXd A2(1,2);
    Eigen::VectorXd b2(1);
    Eigen::MatrixXd D2(1,2);
    Eigen::VectorXd f2(1);

    A2 << 0, 1;
    b2 << 0;
    D2 << 0, 1;
    f2 << -20;

    Task test2(A2, b2, D2, f2);
    
    /*
    std::cout << "A: " << A2 <<"\n";
    std::cout << "b: " << b2 << "\n";
    std::cout << "D: " << D2 << "\n";
    std::cout << "f: " << f2 << "\n";
    std::cout << "Prova" << A.transpose()*A << "\n";
    */
    /*
    -> x = w
    Dx - f <= v -> [ 1 ] x - [-2 ] <= v -> x + 2 <= v -> x <= -2 +v
                   [-1 ]     [-10]      -> -x +10 <= v -> x >= +10 +v
    -> x - 2 <= v
    -> -x + 1 <= v
    */

    std::vector<Task> task_vec = {test1, test2};
    std::cout <<"Il vettore di task è: " << task_vec[0] << "\n" << task_vec[1] <<"\n\n";
    HO ho(task_vec, 2);
    for (Task i: ho.get_HO()){
        std::cout <<i <<"\n";
    }
    task_vec = {test2};
    VectorXd x_opt2(2);
    x_opt2 = ho.solve_ho();
    std::cout <<"Soluzione trovata: " <<x_opt2;

    Eigen::MatrixXd A3(1,1);
    Eigen::VectorXd b3(1);
    Eigen::MatrixXd D3(1,1);
    Eigen::VectorXd f3(1);

    A3(0,0) = 1;
    b3(0) = 0;
    D3(0,0) = 1;
    f3(0) = -20;
    /*
    minimze x^2 + v^2
    s.t     x - 20 <= v
    */

    Task test3(A3, b3, D3, f3);
    std::cout <<test3 << "\n";
    Eigen::VectorXd x_opt3(2);
    x_opt3 = test3.solve_QP();
    std::cout << "Soluzione task 1 singolo: " << x_opt3 <<"\n";




    int state_dim = 8;
    int contact_forces_dim = 1;
    int mpc_step_horizon = 4;
    int joint_dim = state_dim-6;

    Eigen::MatrixXd A = MatrixXd::Zero(0, 2*state_dim + state_dim + contact_forces_dim);
    Eigen::VectorXd b = VectorXd::Zero(0);

    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*(joint_dim+contact_forces_dim), mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim) );
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*(joint_dim+contact_forces_dim));

    Eigen::MatrixXd I_aug = MatrixXd::Zero(joint_dim+contact_forces_dim, joint_dim+contact_forces_dim);
    I_aug.topLeftCorner(joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);

    for (int i = 0; i < mpc_step_horizon; ++i)
    {
        D.block(i * I_aug.rows(), i * I_aug.cols(), I_aug.rows(), I_aug.cols()) = I_aug;
    }

    std::cout <<"I_aug:\n" <<I_aug <<"\n";
    std::cout <<"La matrice diagonale a blocchi è:\n" << D <<"\n";


    Eigen::MatrixXd A4(1,10);
    Eigen::VectorXd b4(1);
    Eigen::MatrixXd D4(1,10);
    Eigen::VectorXd f4(1);

    A4 << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    b4 << 0;
    D4 << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    f4 << -20;

    Task test4(A4, b4, D4, f4);

    std::cout <<test4 << "\n";
    Eigen::VectorXd x_opt4(10);
    std::vector<Task> task_vector(1);
    task_vector[0]=test4;
    HO hier_opt1 = HO(task_vector, 1);
    x_opt4 = hier_opt1.solve_ho();
    std::cout << "Soluzione task 4 singolo: " << x_opt4 <<"\n";

}