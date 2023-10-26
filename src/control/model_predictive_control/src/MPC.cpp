#include "model_predictive_control/MPC.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

using namespace Eigen;
using namespace pinocchio;

/*
Model Predictive Control based on SOLO12 robot
*/

MPC::MPC(std::string robot_name="solo12", double dT=1.0/400):robot(robot_name){
    this->robot_name = robot_name;
    this->dT = dT;
    this->mpc_step_horizon = 1;
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet"};

    // ***************************************************************
    this->task_request = {"dynamic_constraint", "motion_tracking_base", "motion_tracking_feet"};
    // ****************************************************************
    
    
    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    for (int j=0; j<mpc_step_horizon; j++){
        //optimal_input[j] = VectorXd::Zero(2*(this->robot.get_state_dim()-6));
        optimal_input[j] = VectorXd::Zero(this->robot.get_state_dim());
    }
    this->robot.set_optimal_input(optimal_input);
    
}

std::vector<Eigen::VectorXd> MPC::solve_MPC(Eigen::VectorXd q_, Eigen::VectorXd q_dot_, GeneralizedPosesWithTime gen_poses){

    Eigen::VectorXd q = q_;
    Eigen::VectorXd q_dot = q_dot_;
    robot.set_q(q);
    robot.set_qdot(q_dot);

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = this->robot.get_contact_feet_dim()*3;

    robot.compute_EOM(q, q_dot);
    Eigen::VectorXd xi_opt = VectorXd::Zero(q_dim + v_dim + joint_dim + contact_forces_dim);

    std::vector<Task> task_vec(task_request.size());
    task_vec = create_tasks(task_request, gen_poses, q, q_dot);

    HO hierarchical_optimization(task_vec, task_vec.size());

    xi_opt = hierarchical_optimization.solve_ho(task_request);

    //std::cout << "Soluzione ottima:\n" <<xi_opt <<"\n" <<std::endl;

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_torque(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);

    Eigen::MatrixXd Jc = MatrixXd::Zero(contact_forces_dim, v_dim);
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    robot.compute_terms(q);

    std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
    // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }
    robot.get_Jc(Jc, q, contact_feet_names);
    //std::cout <<"Jc dopo l'ottimizzazione:\n" <<Jc <<"\n" <<std::endl;

    Eigen::VectorXd tau_i = VectorXd::Zero(joint_dim);
    Eigen::VectorXd f_i = VectorXd::Zero(contact_forces_dim);

    std::vector<Eigen::VectorXd> optimal_input_temp(mpc_step_horizon);

    for (int i=0; i<mpc_step_horizon; i++){
        tau_i.setZero();
        f_i.setZero();

        tau_i = xi_opt.segment(q_dim + v_dim + i*(q_dim+v_dim+joint_dim+contact_forces_dim), joint_dim);
        f_i = xi_opt.segment(q_dim+v_dim+joint_dim + i*(q_dim+v_dim+joint_dim+contact_forces_dim), contact_forces_dim);
        optimal_torque[i] = tau_i;
        optimal_contact_forces[i] = f_i;
        //optimal_input[i] = tau_i + Jc.rightCols(joint_dim).transpose()*f_i;

        optimal_input_temp[i] =  tau_i + Jc.rightCols(joint_dim).transpose()*f_i;
        Eigen::VectorXd optimal_input_ext = Eigen::VectorXd::Zero(robot.get_robot_model().nv); 
        optimal_input_ext.tail(joint_dim) = optimal_input_temp[i];
        optimal_input[i] = optimal_input_ext;

        joint_state_propagated = robot.compute_dynamics(q, q_dot, optimal_input[i], dT);
        q = joint_state_propagated[0];
        q_dot = joint_state_propagated[1];
        robot.compute_terms(q);

        contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }
        robot.get_Jc(Jc, q, contact_feet_names);
    }
    
    std::cout <<"******************** DESIRED TASK LIST:********************\n";
    for(int i=0; i<task_request.size(); i++){
        std::cout <<task_request[i] <<"\n";
    }
    /*
    std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal torque at step " << i <<" :\n" <<optimal_torque[i] <<"\n" <<std::endl;
    }
    std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal contact forces at step " << i <<" :\n" <<optimal_contact_forces[i] <<"\n" <<std::endl;
    }
    std::cout <<"\n";
    
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal input to the joints at step " << i <<" :\n" <<optimal_input[i] <<"\n" <<std::endl;
    }
    std::cout <<"\n" <<std::endl;
*/
    robot.set_optimal_input(optimal_input);
    //return optimal_input;
    return optimal_input_temp;
}

// CREATION OF EACH TASK INSIDE THE "task_request" VECTOR

std::vector<Task> MPC::create_tasks(std::vector<std::string> task_request, GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q, Eigen::VectorXd q_dot){

    std::vector <Task> task_vec(0);

    for (long unsigned int i=0; i<=task_request.size() -1; i++){
        std::string task_req = task_request[i];
        std::cout <<"Requested task:\n" <<task_req <<"\n" <<std::endl;
        switch(resolveOption(task_req)){
            case 0:{     // task_req = dynamic_constraint

                std::cout <<"\n******************** Starting dynamic constraint ********************\n" <<std::endl;
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, dynamic_constraint(gen_poses, robot.get_robot_model(), robot.get_robot_data(), q, q_dot));
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task dinamica:\n" <<task_vec.size() <<"\n" <<std::endl;
            }
            break;

            case 1:{     // task_req = torque_limits_constraint

                std::cout <<"\n******************** Starting torque limits constraint ********************\n" <<std::endl;
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, torque_limits_constraint());
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task coppie:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 2:{     // task_req = friction_constraint
                std::cout <<"\n******************** Starting friction constraint ********************\n" <<std::endl;
                task_vec.push_back(friction_constraint(gen_poses));
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task attrito:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 3:{     // task_req = motion_tracking    //GeneralizedPose pose_gen;
                std::cout <<"\n******************** Starting motion tracking constraint ********************\n" <<std::endl;
                //task_vec.push_back(motion_tracking_constraint(gen_poses));
                //task_vec.push_back(motion_tracking_base_constraint(gen_poses, q, q_dot));
                //task_vec.push_back(motion_tracking_swing_feet_constraint(gen_poses));
                
            }
            break;

            case 4:{    // task_req = motion_tracking_base
                std::cout <<"\n******************** Starting base motion tracking constraint ********************\n" <<std::endl;
                task_vec.push_back(motion_tracking_base_constraint(gen_poses, q, q_dot));
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task tracking base:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 5:{    // task_req = motion_tracking_swing_feet
                std::cout <<"\n******************** Starting feet motion tracking constraint ********************\n" <<std::endl;
                task_vec.push_back(motion_tracking_swing_feet_constraint(gen_poses));
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task tracking piedi:\n" <<task_vec.size() <<"\n";
            }
            break;

            default:
                throw std::invalid_argument("Invalid task request");
            break;

        }
    }
    return task_vec;
}

int MPC::resolveOption(std::string task_name){
    if (task_name == "dynamic_constraint") return 0;
    else if(task_name == "torque_limits_constraint") return 1;
    else if(task_name == "friction_constraint") return 2;
    else if(task_name == "motion_tracking") return 3;
    else if(task_name == "motion_tracking_base") return 4;
    else if(task_name == "motion_tracking_feet") return 5;
    else return 100;
}

/*
// PROVA DI RISCRITTURA DYNAMIC CONSTRAINT ***************************************************
Task MPC::dynamic_constraint(GeneralizedPosesWithTime gen_poses, Model robot_model, Data data, Eigen::VectorXd q_, Eigen::VectorXd q_dot_){
    
    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim-6;
    int contact_forces_dim = 3*this->robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int input_dim = joint_dim+contact_forces_dim;

    int dim_A = q_dim + v_dim;
    Eigen::VectorXd q = q_;
    //std::cout <<"Stampa q\n" <<q <<"\n" <<std::endl;;
    Eigen::VectorXd q_dot = q_dot_;
    //std::cout <<"Stampa q_dot\n" <<q_dot <<"\n" <<std::endl;
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    //std::cout <<"Stampa tau\n" <<tau <<"\n" <<std::endl;

    pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
    Eigen::MatrixXd ddq_dq = data.ddq_dq.topRows(6);
    Eigen::MatrixXd ddq_dv = data.ddq_dv.topRows(6);

    Eigen::MatrixXd A0 = MatrixXd::Zero(12, dim_A);
    Eigen::MatrixXd Bi = MatrixXd::Zero(12, input_dim);
    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), v_dim);
    Eigen::MatrixXd S = MatrixXd::Zero(joint_dim, v_dim);
    S.rightCols(joint_dim).setIdentity();

    Eigen::MatrixXd A = MatrixXd::Zero(13, dim_A+joint_dim+contact_forces_dim);
    Eigen::VectorXd b = VectorXd::Zero(13);
    Eigen::MatrixXd D = MatrixXd::Zero(1, dim_A+joint_dim+contact_forces_dim);
    Eigen::VectorXd f = VectorXd::Zero(1);

    A0.block(0, q_dim, 6, 6) = MatrixXd::Identity(6, 6);
    A0.bottomLeftCorner(6, v_dim) = ddq_dq;
    A0.bottomRightCorner(6, v_dim) = ddq_dv;
    std::cout <<"Stampa A0\n" <<A0 <<"\n" <<std::endl;

    // Construction of b vector
    Eigen::VectorXd x0 = VectorXd::Zero(dim_A);
    x0.head(q_dim) = q;
    x0.tail(v_dim) = q_dot;
    b = A0*x0;
    std::cout <<"b: \n" <<b <<std::endl;

    // Insert B0
    std::cout <<"Stampa Minv: \n" <<data.Minv <<"\n" <<std::endl;
    std::cout <<"Stampa M\n" <<data.M <<"\n" <<std::endl;

    robot.compute_terms(q);

    std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
    // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }

    robot.get_Jc(Jc, q, contact_feet_names);
    //std::cout <<"Stampa Jc:\n" <<Jc <<"\n";
    //Bi.topLeftCorner(v_dim, joint_dim) = data.Minv.rightCols(joint_dim);
    //Bi.bottomRightCorner(v_dim, contact_forces_dim) = data.Minv*Jc.transpose();
    Eigen::MatrixXd prova = data.Minv*Jc.transpose();   // 18x18 * 18*12 = 18*12
    Bi.bottomRightCorner(6, joint_dim+contact_forces_dim) << data.Minv.block(0,6,6,joint_dim), prova.topRows(6);
    std::cout <<"Bi\n" <<Bi <<std::endl;
    A.block(0, dim_A, 12, input_dim) = -Bi;

    A.block(0, 0, 7, 7) = MatrixXd::Identity(7, 7);
    A.block(7, q_dim, 6, 6) = MatrixXd::Identity(6,6);

    //std::cout <<"Dimensioni matrice A dinamica:\n" <<A.rows() <<"   " <<A.cols() <<"\n";
    Task dynamic_constraints(A, b, D, f);
    //std::cout <<"\nDynamic constraint: \n" <<dynamic_constraints <<"\n" <<std::endl;
    return dynamic_constraints;
}
*/
// *******************************************************************************************************************************************************

// ##################################################################################################################################### 
// #                                                     DYNAMIC CONSTRAINT                                                            #
// #####################################################################################################################################


Task MPC::dynamic_constraint(GeneralizedPosesWithTime gen_poses, Model robot_model, Data data, Eigen::VectorXd q_, Eigen::VectorXd q_dot_){
    /*
        x_(k+1) = A_k*x_k + B_k*u_k
        x_1 = A_0*x_0 + B_0*x_0
                    ...
        x_(N-1) = A_(N-2)*x_(N-2) + B_(N-2)*u_(N-2)
        [ I   0   0 ... 0 -B0  0   ...  0] [  x1   ]    [A0*x0]
        [-A1  I   0 ... 0   0 -B1  ...  0] [  x2   ]    [  0  ]
        [ 0  -A2  I ... 0   0  0   ...   ] [  ...  ]    [ ... ]
        [                          ...   ] [x_(N-1)] =  [ ... ]
        [0    0   0     I   0  ...  0 -B1] [  u0   ]    [ ... ]
                                           [  ...  ]    [ ... ]
                                           [u_(N-2)]    [  0  ]

        Dynamic: M(q,q_dot)*q_ddot + h(q, q_dot) = S'*tau + J_c'*fc

        Relationship between derivative of quaternion and angular velocity:
        quat = q0 + q1*i + q2*j + q3*k,        w = 0 + wx*i + wy*j + wz*k
        quat = [q0, q1, q2, q3] (q0=scalar part), w = [0, wx, wy, wz]

        quat_dot = 0.5 * W (*) quat   (*) = quaternion product

        q0_dot = -0.5*(wx*q1+wy*q2+wz*q3)       [q0_dot]         [ q0 -q3  q2] [wx]
        q1_dot = 0.5*(wx*q0-wy*q3+wz*q2)    =>  [q1_dot] =  0.5* [ q3  q0 -q1] [wy]
        q2_dot = 0.5*(wy*q0-wz*q1+wx*q3)        [q2_dot]         [-q2  q1  q0] [wz]
        q3_dot = 0.5*(wz*q0+wy*q1-wx*q2)        [q3_dot]         [-q1 -q2 -q3]

        
                              [I    0   0] * [      v_b     ]
        q_dot != v => q_dot = [0    Q   0]   [      w_b     ] = M*v
                              [0    0   I]   [ q_dot_joints ]

            [  q  ]   [x1]             [x1_dot]   [q_dot ]   [                M*x2                  ]   [f1(x, u)]
        x = [     ] = [  ]  -> x_dot = [      ] = [      ] = [                                      ] = [        ] = f(x, u)
            [  v  ]   [x2]             [x2_dot]   [q_ddot]   [ Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2)) ]   [f2(x, u)]

        Linearizing around xc, uc

        x_dot = 0 + d f(x,u)/dx |(xc, uc) *(x-xc) + d f(x,u)/du |(xc, uc) *(u-uc)
        x-xc = x_hat,   u-uc=u_hat
        d f1/dx = [d f1/dx1, d f2/dx2]
        d f2/dx = [d f2/dx1, d f2/dx2]

        d f1/dx1 = 0,       d f1/dx2 = I
        d f2/dx1 = d/dq [Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dq
        d f2/dx2 = d/dq_dot [Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dv

                   [  0        I  ]
        => df/dx = [              ] = A
                   [ddq_dq  ddq_dv]

        d f1/du = [d_f1/d_tau, d_f1/d_fc] = [0, 0]
        d f2/du = [d_f2/d_tau, d_f2/d_fc] = [Minv*S', Minv*Jc']

                    [   0           0   ]
        => df/du =  [                   ] = B
                    [Minv*S'    Minv*Jc']

        -> x_dot = A*x_hat + B*u_hat
        */
    //  Ricordarsi di chiudere il commento qui

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim-6;
    int contact_forces_dim = 3*this->robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    //int state_dim = this->robot.get_state_dim();
    int input_dim = joint_dim+contact_forces_dim;
    //std::cout <<"Dimension of the configuration vector: " <<this->robot.get_robot_model().nq <<"\n" <<std::endl;
    //std::cout <<"Dimension of the velocity vector: " <<this->robot.get_robot_model().nv <<"\n" <<std::endl;

    //int dim_A = joint_dim*2;
    int dim_A = q_dim + v_dim;
    Eigen::VectorXd q = q_;
    //std::cout <<"Stampa q\n" <<q <<"\n" <<std::endl;;
    Eigen::VectorXd q_dot = q_dot_;
    //std::cout <<"Stampa q_dot\n" <<q_dot <<"\n" <<std::endl;
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    //std::cout <<"Stampa tau\n" <<tau <<"\n" <<std::endl;

    // Quaternion:  q = qw*1 + qx*i + qy*j + qz*k
    auto qw = q(6);
    auto qx = q(3);
    auto qy = q(4);
    auto qz = q(5);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
    Q.row(0) << qw, -qz, qy;
    Q.row(1) << qz, qw, -qx;
    Q.row(2) << -qy, qx, qw;
    Q.row(3) << -qx, -qy, -qz;

    pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
    Eigen::MatrixXd ddq_dq = data.ddq_dq;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.
    //std::cout <<"ddq_dq, of dimensions: " <<data.ddq_dq.rows() <<" rows, " <<data.ddq_dq.cols() <<" cols:\n" <<data.ddq_dq <<"\n" <<std::endl;
    //std::cout <<"ddq_dv, of dimensions: " <<data.ddq_dv.rows() <<" rows, " <<data.ddq_dv.cols() <<" cols:\n" <<data.ddq_dv <<"\n" <<std::endl;

    Eigen::MatrixXd A0 = MatrixXd::Zero(dim_A, dim_A);
    Eigen::MatrixXd Bi = MatrixXd::Zero(dim_A, input_dim);
    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), v_dim);
    Eigen::MatrixXd S = MatrixXd::Zero(joint_dim, v_dim);
    S.rightCols(joint_dim).setIdentity();

    Eigen::MatrixXd A = MatrixXd::Zero(dim_A*mpc_step_horizon, mpc_step_horizon*(dim_A+joint_dim+contact_forces_dim));
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*dim_A);
    Eigen::MatrixXd D = MatrixXd::Zero(1, mpc_step_horizon*(dim_A+joint_dim+contact_forces_dim));
    Eigen::VectorXd f = VectorXd::Zero(1);

    A0.block(0, q_dim, 3, 3) = Eigen::MatrixXd::Identity(3,3);
    A0.block(3, q_dim+3, 4, 3) = Q;
    A0.block(7, q_dim+6, joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
    A0.bottomLeftCorner(v_dim, v_dim) = ddq_dq;
    A0.bottomRightCorner(v_dim, v_dim) = ddq_dv;
    //std::cout <<"Stampa A0\n" <<A0 <<"\n" <<std::endl;

    // Construction of b vector
    Eigen::VectorXd x0 = VectorXd::Zero(dim_A);
    x0.head(q_dim) = q;
    x0.tail(v_dim) = q_dot;
    b.head(dim_A) = A0*x0;

    // Insert B0
    //std::cout <<"Stampa Minv: \n" <<data.Minv <<"\n" <<std::endl;
    //std::cout <<"Stampa M\n" <<data.M <<"\n" <<std::endl;

    robot.compute_terms(q);

    std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
    // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }

    robot.get_Jc(Jc, q, contact_feet_names);
    //std::cout <<"Stampa Jc:\n" <<Jc <<"\n";
    //Bi.topLeftCorner(v_dim, joint_dim) = data.Minv.rightCols(joint_dim);
    //Bi.bottomRightCorner(v_dim, contact_forces_dim) = data.Minv*Jc.transpose();
    Bi.bottomRightCorner(v_dim, joint_dim+contact_forces_dim) << data.Minv*S.transpose(), data.Minv*Jc.transpose();
    A.block(0, dim_A, dim_A, input_dim) = -Bi;

    for (int j=0; j<mpc_step_horizon; j++){
        A.block(j * dim_A, j * dim_A, dim_A, dim_A) = MatrixXd::Identity(dim_A, dim_A);
    }

    Eigen::MatrixXd Ai = MatrixXd::Zero(dim_A, dim_A);
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    for (int j=0; j<mpc_step_horizon-1; j++){
        Ai.setZero();
        Q.setZero();
        tau = robot.get_optimal_input()[j];
        joint_state_propagated = robot.compute_dynamics(q, q_dot, tau, dT);
        q = joint_state_propagated[0];
        q_dot = joint_state_propagated[1];

        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
        ddq_dq = data.ddq_dq;
        ddq_dv = data.ddq_dv;

        Ai.block(0, q_dim, 3, 3) = Eigen::MatrixXd::Identity(3,3);
        Ai.block(3, q_dim+3, 4, 3) = Q;
        Ai.block(7, q_dim+6, joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
        Ai.bottomLeftCorner(v_dim, v_dim) = ddq_dq;
        Ai.bottomRightCorner(v_dim, v_dim) = ddq_dv; 


        A.block(j*dim_A + dim_A, j*dim_A + dim_A, dim_A, dim_A) = Ai;
        std::cout <<"A:\n" <<A <<"\n";

        robot.compute_terms(q);
        
        contact_feet_names = gen_poses.generalized_poses_with_time[j].gen_pose.contact_feet_names;
        // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }

        robot.get_Jc(Jc, q, contact_feet_names);
        Bi.setZero();
        //Bi.topLeftCorner(v_dim, v_dim) = data.Minv;
        Bi.topLeftCorner(v_dim, joint_dim) = data.Minv * S.transpose();
        //std::cout <<"Stampa Bi\n" <<Bi <<"\n";
        Bi.bottomRightCorner(v_dim, contact_forces_dim) = data.Minv*Jc.transpose();
        A.block(j*dim_A + dim_A, j*input_dim + mpc_step_horizon*dim_A, dim_A, input_dim ) = -Bi;
    }

    //std::cout <<"Dimensioni matrice A dinamica:\n" <<A.rows() <<"   " <<A.cols() <<"\n";
    Task dynamic_constraints(A, b, D, f);
    //std::cout <<"\nDynamic constraint: \n" <<dynamic_constraints <<"\n" <<std::endl;
    return dynamic_constraints;
}

// ######################################################################################################################################
//                                                    TORQUE LIMITS CONSTRAINT                                                          #
// ######################################################################################################################################

Task MPC::torque_limits_constraint(){
    /*
        tau <= u.b
        state = [q', q_dot', tau', fc']'
        [0 ... 0 I^A 0 ... 0] [     [q_1', qdot_1']'     ]
        [0 ... 0 0 I^A ... 0] [         [...]            ]
        [         ...       [ [ [q_(N-1)', qdot_(N-1)']' ]          I^A = [I, 0]
        [0 ... 0 0 0 ... I^A] [     [tau_1', fc_1']'     ]                [0, 0]
                              [         [...]            ]
                              [ [tau_(N-1)', fc_(N-1)']  ]
    */

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;

    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (q_dim + v_dim + joint_dim + contact_forces_dim);
    
    Eigen::MatrixXd A = MatrixXd::Zero(1, cols);
    Eigen::VectorXd b = VectorXd::Zero(1);
    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*joint_dim, cols);
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*joint_dim);

    for(int i = 0; i<mpc_step_horizon; i++){
        D.block(i*joint_dim, q_dim+v_dim + i*cols, joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
    }
    for(int i=0; i<mpc_step_horizon; i++){
        D.block(i*joint_dim+joint_dim, q_dim+v_dim + i*cols, joint_dim, joint_dim) = -MatrixXd::Identity(joint_dim, joint_dim);
    }

    f.head(mpc_step_horizon*(joint_dim)) = VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_max();
    f.tail(mpc_step_horizon*(joint_dim)) = -VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_min();

    Task torque_limits(A, b, D, f);
    //std::cout <<"Task torque constraint\n" <<torque_limits <<"\n";
    return torque_limits;
    
}

Task MPC::motion_tracking_constraint(GeneralizedPosesWithTime gen_poses){

    /*
        xi = xi^des

        [I 0 ... 0 0 ... 0] [     [q_1', qdot_1']'     ]    [ x_(1)^des ]
        [0 I ... 0 0 ... 0] [         [...]            ]    [ x_(2)^des ]
        [   ...    0 ... 0] [ [q_(N-1)', qdot_(N-1)']' ] =  [    ...    ]
        [0 0 ... I 0 ... 0] [     [tau_1', fc_1']'     ]    [x_(N-1)^des]
                            [         [...]            ]
                            [ [tau_(N-1)', fc_(N-1)']' ]
   */
   //std::cout <<"Stampa motion tracking\n" <<std::endl;
    //int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(mpc_step_horizon*(2*joint_dim), cols);
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*(2*joint_dim));

    Eigen::MatrixXd D = MatrixXd::Zero(1, cols);
    Eigen::VectorXd f = VectorXd::Zero(1);

    A.leftCols(mpc_step_horizon*(2*joint_dim)) = MatrixXd::Identity(mpc_step_horizon*(2*joint_dim), mpc_step_horizon*(2*joint_dim));

    Eigen::VectorXd joint_des_pos = VectorXd::Zero(robot.get_state_dim());
    Eigen::VectorXd joint_des_pos_prev = VectorXd::Zero(robot.get_state_dim());
    Eigen::VectorXd joint_des_vel = VectorXd::Zero(robot.get_state_dim());

        // #################################################################################################
        joint_des_pos << 0.0, 0.3, -0.6, 0.0, 0.3, -0.6, 0.0, -0.3, 0.6, 0.0, -0.3, 0.6;
        // ################################################################################################

    std::cout << "A:\n" <<A <<"\n" <<std::endl;
    for (int i=0; i<mpc_step_horizon; i++){
        //std::cout <<"i motion track: " <<i <<std::endl;
        //joint_des_pos = robot.clik_alg(gen_poses.generalized_poses_with_time[i].gen_pose, robot.get_robot_model(), robot.get_robot_data());
        std::cout <<"joint_des_pos:\n" <<joint_des_pos <<"\n" <<std::endl;
        //joint_des_vel = (joint_des_pos-joint_des_pos_prev)/dT;
        std::cout <<"joint_des_vel:\n" <<joint_des_vel <<"\n" <<std::endl;
        joint_des_pos_prev = joint_des_pos;
        //std::cout <<"Posizioni desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos <<"\n";
        //std::cout <<"Velocità desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel <<"\n";

        /*
        b.segment(2*i*(joint_dim), joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel;
        */

        b.segment(2*i*(joint_dim), joint_dim) = joint_des_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = joint_des_vel;

    }
    std::cout <<"b: \n" <<b <<"\n" <<std::endl;

     Task motion_tracking(A, b, D, f);
     return motion_tracking;
    
}

// ################################################################################################################################
// #                                                 MOTION TRACKING OF BASE                                                      #
// ################################################################################################################################

/*
        [v_b]                   [vb_ref]
        [   ] = Jb * q_dot  =   [      ]
        [w_b]                   [wb_ref]
*/

Task MPC::motion_tracking_base_constraint(GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){
    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim-6;

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;

    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    std::vector<Eigen::VectorXd> joint_state_propagated(2);

    Eigen::MatrixXd Jb = Eigen::MatrixXd::Zero(6, robot.get_robot_model().nv);
    
    robot.compute_second_order_FK(q, v);
    robot.compute_terms(q);
    robot.get_Jb(Jb);
    //std::cout <<"Jb:\n" <<Jb <<"\n" <<std::endl;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6*mpc_step_horizon, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6*mpc_step_horizon);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(1);

    A.block(0, q_dim, 6, v_dim) = Jb;
    //std::cout <<"base_vel:\n" <<gen_poses.generalized_poses_with_time[0].gen_pose.base_vel <<"\n" <<std::endl;
    b.segment(0, 3) = gen_poses.generalized_poses_with_time[0].gen_pose.base_vel;
    b.segment(3, 3) = gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel;
    //std::cout <<"A:\n" <<A <<"\n" <<std::endl;
    //std::cout <<"b:\n" <<b <<"\n" <<std::endl;

    for (int i=1; i<mpc_step_horizon; i++){
        Jb.setZero();

        tau = robot.get_optimal_input()[i];
        joint_state_propagated = robot.compute_dynamics(q, v, tau, dT);
        q = joint_state_propagated[0];
        v = joint_state_propagated[1];

        robot.compute_second_order_FK(q, v);
        robot.compute_terms(q);
        robot.get_Jb(Jb);

        A.block(i*6, i*q_dim, 6, v_dim) = Jb;

        b.segment(3*i, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_vel;
        b.segment(3*i+3, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel;
    }

    Task motion_tracking(A, b, D, f);
    //std::cout <<"Task motion tracking:\n" <<motion_tracking <<"\n" <<std::endl;
    return motion_tracking;
}

// ################################################################################################################################
// #                                             MOTION TRACKING OF SWING FEET                                                    #
// ################################################################################################################################

Task MPC::motion_tracking_swing_feet_constraint(GeneralizedPosesWithTime gen_poses){

    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim - 6;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*(3*4), mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*12);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(1);

    Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(3*4, robot.get_robot_model().nv);

    for (int i=0; i<mpc_step_horizon; i++){
        Js.setZero();
        //std::cout <<"Dimensione generalized poses:\n" <<gen_poses.generalized_poses_with_time.size() <<"\n" <<std::endl;
        std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        /*
        std::cout <<"contact_feet_names dentro task:\n" <<std::endl;
        for(int k=0; k<contact_feet_names.size(); k++){
            std::cout <<contact_feet_names[k] <<" " <<std::endl;
        }
        */

        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();

        /*
        std::cout <<"short_feet_names dentro task:\n" <<std::endl;
        for(int k=0; k<robot_short_feet_names.size(); k++){
            std::cout <<robot_short_feet_names[k] <<" " <<std::endl;
        }
        */

        for (auto & foot_name : contact_feet_names) {
        //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
        auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

        //int index = std::distance(robot_short_feet_names.begin(), it);
        int index = it - robot_short_feet_names.begin();

        foot_name = robot.get_feet_names()[index];
        //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }

    /*
        std::cout <<"contact_feet_names dentro task dopo il cambio nome:\n" <<std::endl;
        for(int k=0; k<contact_feet_names.size(); k++){
            std::cout <<contact_feet_names[k] <<" " <<std::endl;
        }
        */

        std::vector<std::string> swing_feet_names;
        swing_feet_names = {};
        std::vector<int> swing_feet_index;

        for (size_t j= 0; j < 4; j++) {
            if ( std::find(contact_feet_names.begin(), contact_feet_names.end(), robot_feet_names[j]) == contact_feet_names.end() ) {
                // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing foot.
                swing_feet_index.push_back(j);
                swing_feet_names.push_back(robot_feet_names[j]);
            }
        }

        robot.get_Js(Js, swing_feet_names, swing_feet_index);

        //std::cout <<"\n" <<std::endl;
        //std::cout <<"Js:\n" <<Js <<"\n" <<std::endl;

        for(int k=0; k<swing_feet_index.size(); k++){
        //std::cout <<"swing_feet_index[i]:\n" <<swing_feet_index[k] <<"   " <<std::endl;

        b.segment(i*12 + k*3, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel;
        }

        A.block(i*12, i*v_dim, 12, v_dim) = Js;

        // *******************************************************************************************
        // TODO: IMPLEMENT DYNAMICS AND UPDATING OF JOINT POSITION!!!!!!!!!!!!!!!!!!!!!
        // *******************************************************************************************

    }
    Task motion_tracking_swing_feet(A, b, D, f);
    //std::cout <<"Tracking swing feet:\n" <<motion_tracking_swing_feet <<"\n" <<std::endl;
    return motion_tracking_swing_feet;
}

// ###############################################################################################################################
// #                                             FRICTION CONSTRAINT                                                             #
// ###############################################################################################################################

Task MPC::friction_constraint(GeneralizedPosesWithTime gen_poses){
    /*
    To avoid slipping, the resulting contact forces must be constrained to lie within the friction cones 
    (approximated as pyramid cone for linearity)
    h->heading, l->lateral, n->normal directions, mu->friction coefficient
    (h-n*mu)*f <=0                  [0, 0, 0, h-n*mu    ] [q    ]    [0]
    -(h+n*mu)*f <=0                 [0, 0, 0, -(h+n*mu) ] [q_dot]    [0]     
    (l-n*mu)*f <=0             -->  [0, 0, 0, l-n*mu    ] [tau  ] <= [0]        --> [0  |0 F|] [x] <= d
    -(l+n*mu)*f <=0                 [0, 0, 0, -(l+n*mu) ] [f    ]    [0]                       [u]
    f_min <= n*f <= f_max           [0, 0, 0, n         ]            [f_max]    
                                    [0, 0, 0, -n        ]            [-f_min]

                        [0 ... 0 |0 F|   0   ...   0  ] [     [q_1', qdot_1']'     ]    [d]
                        [0 ... 0   0   |0 F| ...   0  ] [         [...]            ]    [d]
                        [0 ... 0   0     0   ...   0  ] [ [q_(N-1)', qdot_(N-1)']' ] <= [d]
                        [0 ... 0   0     0   ... |0 F|] [     [tau_1', fc_1']'     ]    [d]
                                                        [         [...]            ]    [d]
                                                        [ [tau_(N-1)', fc_(N-1)']' ]    [d]

    */

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim-6;
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = q_dim+v_dim+joint_dim+contact_forces_dim;
    int ncp = robot.get_contact_feet_dim();     // Number of points of contact (legs on the ground)

    Eigen::MatrixXd h = MatrixXd::Zero(ncp, contact_forces_dim);
    Eigen::MatrixXd l = MatrixXd::Zero(ncp, contact_forces_dim);
    Eigen::MatrixXd n = MatrixXd::Zero(ncp, contact_forces_dim);

    for (int i = 0; i < ncp; i++) {
        h.block(i, 3*i, 1, 3) << 1, 0, 0;
        l.block(i, 3*i, 1, 3) << 0, 1, 0;
        n.block(i, 3*i, 1, 3) << 0, 0, 1;
    }
    //std::cout<<"h\n" <<h <<"\n";
    //std::cout<<"l\n" <<l <<"\n";
    //std::cout<<"n\n" <<n <<"\n";

    Eigen::MatrixXd A = MatrixXd::Zero(3*ncp*mpc_step_horizon, mpc_step_horizon*cols);
    Eigen::VectorXd b = VectorXd::Zero(3*ncp*mpc_step_horizon);
    Eigen::MatrixXd D = MatrixXd::Zero(6*ncp*mpc_step_horizon, cols*mpc_step_horizon);
    Eigen::VectorXd f = VectorXd::Zero(6*ncp*mpc_step_horizon);

    Eigen::MatrixXd Ai = MatrixXd::Identity(3*ncp, contact_forces_dim);
    Eigen::MatrixXd Di = MatrixXd::Zero(6*ncp, joint_dim+contact_forces_dim);
    Eigen::VectorXd fi = VectorXd::Zero(6*ncp);
    fi.segment(4*ncp, ncp) = VectorXd::Ones(ncp)*robot.get_f_max();
    fi.segment(5*ncp, ncp) = -VectorXd::Ones(ncp)*robot.get_f_min();
    //std::cout <<"fi:\n" <<fi <<"\n" <<std::endl;

    for (int i = 0; i < mpc_step_horizon; i++) {
        std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        Di.setZero();
        Ai.setIdentity();       // for the swing feet the contact forces must be 0

        // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            //std::cout <<"foot_name: " <<foot_name <<"\n" <<std::endl;
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);

            int index = it - robot_short_feet_names.begin();

            foot_name = robot.get_feet_names()[index];
            //std::cout <<"new foot_name: " <<foot_name <<std::endl;
        }

        for (int j=0; j<static_cast<int>(contact_feet_names.size()); j++) {
            //std::cout <<"contact_feet_names[j]: " <<contact_feet_names[j] <<"\n" <<std::endl;

            auto it = std::find(robot_feet_names.begin(), robot_feet_names.end(), contact_feet_names[j]);
            // find returns an iterator to the first element in range [first, last) that compares equal to val, if no element is found, returns last
            int index = std::distance(robot_feet_names.begin(), it);

            // For each leg in contact (index) populate the Di matrix, the swing legs will have all zeros
            Di.block(index*6, joint_dim, 1, contact_forces_dim) = h.row(index)-n.row(index)*mu;
            Di.block(1+index*6, joint_dim, 1, contact_forces_dim) = -(h.row(index)+n.row(index)*mu);
            Di.block(2+index*6, joint_dim, 1, contact_forces_dim) = l.row(index)-n.row(index)*mu;
            Di.block(3+index*6, joint_dim, 1, contact_forces_dim) =-(l.row(index)+n.row(index)*mu);
            Di.block(4+index*6, joint_dim, 1, contact_forces_dim) = n.row(index);
            Di.block(5+index*6, joint_dim, 1, contact_forces_dim) = -n.row(index);
            //std::cout <<"Di:\n" <<Di <<"\n" <<std::endl;

           Ai.block(3*index, 3*index, 3, 3) = MatrixXd::Zero(3,3);      // setting the equality constraint for the standng feet to null
        }

        D.block(i*(6*ncp), q_dim+v_dim + i*Di.cols(), 6*ncp, Di.cols()) = Di;
        f.segment(i*(6*ncp), 6*ncp) = fi;

        A.block(i*Ai.rows() , q_dim+v_dim+joint_dim + i*Ai.cols(), Ai.rows(), Ai.cols()) = Ai;
        //std::cout <<"D:\n" <<D <<"\n" <<std::endl;
        //std::cout <<"f:\n" <<f <<"\n" <<std::endl;
        //std::cout <<"A:\n" <<A <<"\n" <<std::endl;
    }

    Task friction(A, b, D, f);
    return friction;
}


std::vector<Eigen::VectorXd> MPC::tune_gains(Eigen::VectorXd q_, Eigen::VectorXd q_dot_, GeneralizedPosesWithTime gen_poses){

    int state_dim = robot.get_state_dim();

    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;

    std::vector<Eigen::VectorXd> Kp(number_of_wolves);
    std::vector<Eigen::VectorXd> Kd(number_of_wolves);
    std::vector<Eigen::VectorXd> Ki(number_of_wolves);
    for (int i=0; i<number_of_wolves; i++){
        Kp[i] = Eigen::VectorXd::Random(state_dim)*100;
        Kd[i] = Eigen::VectorXd::Random(state_dim)*100;
        Ki[i] = Eigen::VectorXd::Random(state_dim)*100;
        Kp[i].cwiseAbs();
        Kd[i].cwiseAbs();
        Ki[i].cwiseAbs();
    }

    q = q_;
    q_dot = q_dot_;

    Eigen::VectorXd fit_f = VectorXd::Zero(number_of_wolves);


    for (int i=0; i<number_of_wolves; i++){
        // Compute error
        Eigen::VectorXd error = VectorXd::Zero(2*state_dim);
        Eigen::VectorXd error_prev = VectorXd::Zero(2*state_dim);
        Eigen::VectorXd error_int = VectorXd::Zero(2*state_dim);

        // gen_poses contains the desired pose, not the desired joint pos and vel
        // TODO keep in consideration this
        // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        error.head(state_dim) = q - gen_poses.generalized_poses_with_time[0].gen_pose.joint_pos;
        error.tail(state_dim) = q_dot - gen_poses.generalized_poses_with_time[0].gen_pose.joint_vel;

        std::cout <<"Errore:\n" << error <<"\n";

        fit_f(i) = fit_f(i) + error.transpose()*error;
        std::cout <<"Fit function:\n" <<fit_f <<"\n";

        // Propagate dynamics
        std::vector<Eigen::VectorXd> joint_state_propagated(2);
        Eigen::VectorXd tau = VectorXd::Zero(state_dim);
        for (int j=1; j<mpc_step_horizon; j++){
            tau = Kp[i]*error + Kd[i]*(error-error_prev)/dT + Ki[i]*error_int;
            std::cout <<"Tau PID\n" <<tau <<"\n";
            joint_state_propagated = robot.compute_dynamics(q, q_dot, tau, dT);
            q = joint_state_propagated[0];
            q_dot = joint_state_propagated[1];

        // Error computation
            error_prev = error;
            error.head(state_dim) = q - gen_poses.generalized_poses_with_time[j].gen_pose.joint_pos;
            error.tail(state_dim) = q_dot - gen_poses.generalized_poses_with_time[j].gen_pose.joint_vel;
            std::cout <<"Error:\n" <<error <<"\n";

        // Compute fit function
            fit_f(i) += error.transpose()*error;
            //std::cout <<"Fitting function:\n" <<fit_i <<"\n";
            error_int += error;


        // Search for alpha, beta and delta
            int alpha=0;
            int beta=0;
            int delta=0;

            for (int j=0; j<fit_f.size(); j++){
                if (fit_f(j) < fit_f(alpha)){alpha = j;}
                else if(fit_f(j) < fit_f(beta)){beta = j;}
                else if(fit_f(j) < fit_f(delta)){delta = j;};
            }
            std::cout <<"Alpha: " <<alpha <<" Beta: " <<beta <<" Delta: " <<delta <<"\n";
            for (int j=0; j<number_of_wolves; j++){
                Kp[i] = (Kp[alpha]+2*Kp[beta]+2*Kp[delta]-5*Kp[i])/4;
                Kd[i] = (Kd[alpha]+2*Kd[beta]+2*Kd[delta]-5*Kd[i])/4;
                Ki[i] = (Ki[alpha]+2*Ki[beta]+2*Ki[delta]-5*Ki[i])/4;
            }
        }
    }

    int alpha=0;
    fit_f.minCoeff(&alpha);
    std::vector<Eigen::VectorXd> pid_gains(3);
    pid_gains[0] = Kp[alpha]; 
    pid_gains[1] = Kd[alpha];
    pid_gains[2] = Ki[alpha];

    return pid_gains;
}
