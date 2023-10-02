#include "pid_tuning/pid_tuner.hpp"

namespace pid_tuner{

PIDTuner::PIDTuner(){}

PIDTuner::PIDTuner(Eigen::VectorXd x_des, Eigen::VectorXd x_meas){
    this->x_des = x_des;
    this->x_meas = x_meas;
    this->number_of_wolves = 30;
    this->number_of_iterations = 50;
}

void tune_gains(){
    Eigen::VectorXd error = x_meas - x_des;
    error_hist.pop_front();
    error_hist.push_back(error);
    Eigen::Vectorxd ISE = VectorXd::Zero(error.size());

    for (int k=0; k<number_of_wolves; k++){
        Eigen::VectorXd u = Kp[k]*error + Kd[k]*(error-error_prev)/dT 

        for (int iter=0; iter<step_window; iter++){
            
        }

        // Compute torques
        // Progate dynamics
        // Compute error
        // Cycle

    }

    
}

Eigen::VectorXd PIDTuner::compute_ISE(){
    // ISE = integral_(0:t) e(t)^2 dt
    for (int j=0; j<error.size(); j++){
        for (int i=0; i<step_window; i++){
            ISE(j) = ISE(j) + error_hist[i](j)*error_hist[i](j);
        }
    }
}

}