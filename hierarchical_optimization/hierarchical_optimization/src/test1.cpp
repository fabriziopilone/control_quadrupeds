#include "Task.cpp"
#include <stdio.h>
#include <iostream>
using namespace task;

int main(){
    Eigen::MatrixXd A(1,1);
    Eigen::VectorXd b(1);
    Eigen::MatrixXd D(1,1);
    Eigen::VectorXd f(1);

    A(0,0) = 1;
    b(0) = 0;
    D(0,0) = 1;
    f(0) = 2;

    Task test(A, b, D, f);
    Eigen::VectorXd x_opt(1);
    x_opt = test.solve_QP();
    std::cout << x_opt;
}