#pragma once
#include <Eigen/Dense>
#include "Task.hpp"
using namespace task;

class HO{
    public:
        HO();
        HO(std::vector<task::Task> task_vec, int size);

        std::vector <task::Task> get_HO(){return this->task_vec;};
        Eigen::VectorXd solve_ho();
    
    private:
        Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd&);
        int size;
        std::vector<task::Task> task_vec;
    
};