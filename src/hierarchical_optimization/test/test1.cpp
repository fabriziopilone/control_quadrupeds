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
    
}