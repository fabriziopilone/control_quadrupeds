#include <Eigen/Dense>

class Task{

    public:
        Task();
        Task(Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd D, Eigen::VectorXd f);
        Eigen::VectorXd solve_QP();

        Eigen::MatrixXd get_A();
        Eigen::VectorXd get_b();
        Eigen::MatrixXd get_D();
        Eigen::VectorXd get_f();

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

    private:
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        Eigen::MatrixXd D;
        Eigen::VectorXd f;
};