#pragma once
#include <Eigen/Dense>

namespace task{
    
class Task{

    public:
        Task();
        Task(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd D, Eigen::VectorXd f);
        Eigen::VectorXd solve_QP();

        Eigen::MatrixXd get_A() {return this->A;};
        Eigen::VectorXd get_b() {return this->b;};
        Eigen::MatrixXd get_D() {return this->D;};
        Eigen::VectorXd get_f() {return this->f;};
        double get_reg() {return this->reg;};

        void set_A(Eigen::MatrixXd A){
            this->A= A;
        }
        void set_b(Eigen::VectorXd b){
            this->b = b;
        }
        void set_D(Eigen::MatrixXd D){
            this->D = D;
        }
        void set_f(Eigen::VectorXd f){
            this->f = f;
        }
        void set_task(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd D, Eigen::VectorXd f){
            this->A = A;
            this->b = b;
            this->D = D;
            this->f = f;
        }

        friend auto operator<<(std::ostream& os, Task const& m) -> std::ostream& { 
            return os << m.A;
        }

    private:
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        Eigen::MatrixXd D;
        Eigen::VectorXd f;
        double reg = 1e-06;
};
}