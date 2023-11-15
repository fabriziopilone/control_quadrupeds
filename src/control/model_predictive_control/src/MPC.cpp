#include "model_predictive_control/MPC.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

using namespace Eigen;
using namespace pinocchio;

/*
Model Predictive Control based on SOLO12 robot
*/

MPC::MPC(std::string robot_name="solo12", double dT=1.0/400, int mpc_step_horizon=1):robot(robot_name){
    this->robot_name = robot_name;
    this->dT = dT;
    this->mpc_step_horizon = mpc_step_horizon;
    this->model = robot.get_robot_model();
    this->data = robot.get_robot_data();
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet", "effort_minimization"};

    // ***************************************************************
    this->task_request = {"torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet"};
    //this->task_request = {"dynamic_constraint","torque_limits_constraint", "friction_constraint"};
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet"};
    //this->task_request = {"prova_dinamica", "dynamic_constraint", "torque_limits_constraint", "friction_constraint"};
    // ****************************************************************
    
    std::vector<Eigen::VectorXd> optimal_torques(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    std::vector<Eigen::VectorXd> q_prop(mpc_step_horizon);
    std::vector<Eigen::VectorXd> v_prop(mpc_step_horizon);
    std::vector<Eigen::MatrixXd> Jc_time(mpc_step_horizon);
    for (int j=0; j<mpc_step_horizon; j++){
        q_prop[j] = VectorXd::Zero(this->robot.get_robot_model().nq);
        v_prop[j] = VectorXd::Zero(this->robot.get_robot_model().nv);
        Jc_time[j] = Eigen::MatrixXd::Zero(3*this->robot.get_contact_feet_dim(), this->robot.get_state_dim());
        optimal_torques[j] = VectorXd::Zero(this->robot.get_state_dim()-6);
        optimal_contact_forces[j] = VectorXd::Zero(3*this->robot.get_contact_feet_dim());
        optimal_input[j] = VectorXd::Zero(this->robot.get_state_dim());
    }
    this->q_propagated = q_prop;
    this->v_propagated = v_prop;
    this->Jc_in_time = Jc_time;
    this->robot.set_optimal_torques(optimal_torques);
    this->robot.set_optimal_contact_forces(optimal_contact_forces);
    this->robot.set_optimal_input(optimal_input);
    
}

void MPC::compute_all(Eigen::VectorXd q_, Eigen::VectorXd v_, GeneralizedPosesWithTime gen_poses){
    // Computing all the terms

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    std::vector<Eigen::VectorXd> propagated_state(2);

    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), robot.get_state_dim());
    std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
    std::vector<std::string> robot_feet_names = robot.get_feet_names();

    for (int i=0; i<mpc_step_horizon; i++){
        if(i==0){
            this->q_propagated[i] = q;
            this->v_propagated[i] = v;
            robot.compute_terms(q);

            std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
            std::vector<int> contact_feet_index(0);
            for (auto & foot_name : contact_feet_names) {
                auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
                int index = it - robot_short_feet_names.begin();
                contact_feet_index.push_back(index);
                foot_name = robot.get_feet_names()[index];
            }
            robot.get_Jc(Jc, q, contact_feet_names, contact_feet_index);
            Jc_in_time[0] = Jc;
        }
        else{
            propagated_state = robot.compute_dynamics(q, v, tau, dT);
            q = propagated_state[0];
            v = propagated_state[1];
            tau = this->robot.get_optimal_input()[i];
            this->q_propagated[i] = q;
            this->v_propagated[i] = v;
            robot.compute_terms(q);

            Jc.setZero();
            std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
            std::vector<int> contact_feet_index(0);
            for (auto & foot_name : contact_feet_names) {
                auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
                int index = it - robot_short_feet_names.begin();
                contact_feet_index.push_back(index);
                foot_name = robot.get_feet_names()[index];
            }
            robot.get_Jc(Jc, q, contact_feet_names, contact_feet_index);
            Jc_in_time[i] = Jc;
        }
    }
}

std::vector<Eigen::VectorXd> MPC::solve_MPC(Eigen::VectorXd q_, Eigen::VectorXd q_dot_, GeneralizedPosesWithTime gen_poses){

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = q_dot_;

    int q_dim = model.nq;
    int v_dim = model.nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = this->robot.get_contact_feet_dim()*3;

    compute_all(q, v, gen_poses);
    robot.compute_EOM(q, v);
    Eigen::VectorXd xi_opt = VectorXd::Zero(q_dim + v_dim + joint_dim + contact_forces_dim);

    std::vector<Task> task_vec(task_request.size());
    task_vec = create_tasks(task_request, gen_poses, q, v);

    HO hierarchical_optimization(task_vec, task_vec.size());

    xi_opt = hierarchical_optimization.solve_ho(task_request);

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_torques(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);
    Eigen::VectorXd tau_i = VectorXd::Zero(joint_dim);
    Eigen::VectorXd f_i = VectorXd::Zero(contact_forces_dim);

    tau_i = xi_opt.segment((q_dim + v_dim)*mpc_step_horizon, joint_dim);
    f_i = xi_opt.segment((q_dim+v_dim)*mpc_step_horizon+joint_dim, contact_forces_dim);
    optimal_torques[0] = tau_i;
    optimal_contact_forces[0] = f_i;

    Eigen::VectorXd optimal_input_ext = Eigen::VectorXd::Zero(model.nv); 
    optimal_input_ext.tail(joint_dim) = optimal_torques[0];
    optimal_input[0] = optimal_input_ext;

    for (int i=1; i<mpc_step_horizon; i++){
        tau_i.setZero();
        f_i.setZero();

        tau_i = xi_opt.segment((q_dim + v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim);
        f_i = xi_opt.segment((q_dim+v_dim+joint_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), contact_forces_dim);
        optimal_torques[i] = tau_i;
        optimal_contact_forces[i] = f_i;

        optimal_input_ext.tail(joint_dim) = optimal_torques[i];
        optimal_input[i] = optimal_input_ext;
    }
    
    // std::cout <<"******************** DESIRED TASK LIST:********************\n";
    for(int i=0; i<task_request.size(); i++){
        // std::cout <<task_request[i] <<"\n";
    }
    
    // std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal torque at step " << i <<" :\n" <<optimal_torques[i] <<"\n" <<std::endl;
    }
    // std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal contact forces at step " << i <<" :\n" <<optimal_contact_forces[i] <<"\n" <<std::endl;
    }
    
    pinocchio::FrameIndex foot_id = model.getFrameId("FL_FOOT");
    Eigen::VectorXd foot_pos = data.oMf[foot_id].translation();
    std::cout <<"foot_pos:\n" <<foot_pos <<std::endl;

    robot.set_optimal_torques(optimal_torques);
    robot.set_optimal_contact_forces(optimal_contact_forces);
    robot.set_optimal_input(optimal_input);
    //return optimal_input;
    return optimal_torques;
}

// CREATION OF EACH TASK INSIDE THE "task_request" VECTOR

std::vector<Task> MPC::create_tasks(std::vector<std::string> task_request, GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q, Eigen::VectorXd q_dot){

    std::vector <Task> task_vec(0);

    for (long unsigned int i=0; i<=task_request.size() -1; i++){
        std::string task_req = task_request[i];

        switch(resolveOption(task_req)){
            case 7:{
                task_vec.push_back(prova_dinamica());
            }
            break;

            case 0:{     // task_req = dynamic_constraint

                std::cout <<"\n******************** Starting dynamic constraint ********************\n" <<std::endl;
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, dynamic_constraint(gen_poses, model, data, q, q_dot));
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
                // std::cout <<"\n******************** Starting motion tracking constraint ********************\n" <<std::endl;
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
                task_vec.push_back(motion_tracking_swing_feet_constraint(model, data, gen_poses, q, q_dot));
                //std::cout <<"Dimensione vettore dei task dopo aver inserito il task tracking piedi:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 6:{    // task_req = effort_minimization
                // std::cout << "\n******************** Starting effort minimization ********************\n" <<std::endl;
                task_vec.push_back(effort_minimization());
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
    else if(task_name == "effort_minimization") return 6;
    else if(task_name == "prova_dinamica") return 7;
    else return 100;
}

Task MPC::prova_dinamica(){
    int q_dim = model.nq;
    int v_dim = model.nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = robot.get_contact_feet_dim()*3;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*joint_dim, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*joint_dim);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    Eigen::VectorXd tau_equilibrio = Eigen::VectorXd::Zero(12);
    tau_equilibrio << -0.00775627, -0.16386, 0.187965, 0.0111173, -0.162693, 0.186111, -0.00761905, 0.163021, -0.188014, 0.0110333, 0.1635, -0.186075;


    for(int i=0; i<mpc_step_horizon; i++){
        A.block(i*joint_dim, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_forces_dim), joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
        b.segment(i*joint_dim, joint_dim) = tau_equilibrio;
    }
    Task prova_dinamica(A, b, D, f);
    std::cout <<"Task prova dinamica:\n" <<prova_dinamica <<std::endl;
    return prova_dinamica;
}

// *******************************************************************************************************************************************************

// ##################################################################################################################################### 
// #                                                     DYNAMIC CONSTRAINT                                                            #
// #####################################################################################################################################
    /*
        x_dot = f(x, u) -> x_(k+1) = x_k + dT*f(x_k, u_k)

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

        Dynamic: M(q, v)*v_dot + h(q, v) = S'*tau + J_c'*fc

        Relationship between derivative of quaternion and angular velocity:
        quat = q0 + q1*i + q2*j + q3*k,        w = 0 + wx*i + wy*j + wz*k
        quat = [q0, q1, q2, q3] (q0=scalar part), w = [0, wx, wy, wz]

        quat_dot = 0.5 * W (*) quat   (*) = quaternion product

        q0_dot = -0.5*(wx*q1+wy*q2+wz*q3)       [q1_dot]         [ q0 -q3  q2] [wx]
        q1_dot = 0.5*(wx*q0-wy*q3+wz*q2)    =>  [q2_dot] =  0.5* [ q3  q0 -q1] [wy]
        q2_dot = 0.5*(wy*q0-wz*q1+wx*q3)        [q3_dot]         [-q2  q1  q0] [wz]
        q3_dot = 0.5*(wz*q0+wy*q1-wx*q2)        [q0_dot]         [-q1 -q2 -q3]

        
                              [I    0   0] * [      v_b     ]
        q_dot != v => q_dot = [0    Q   0]   [      w_b     ] = G*v
                              [0    0   I]   [ q_dot_joints ]

            [  q  ]   [x1]             [x1_dot]   [q_dot]   [                G*x2                  ]   [f1(x, u)]
        x = [     ] = [  ]  -> x_dot = [      ] = [     ] = [                                      ] = [        ] = f(x, u)
            [  v  ]   [x2]             [x2_dot]   [v_dot]   [ Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2)) ]   [f2(x, u)]

        x_(k+1) = x_k + dT*f(x_k, u_k) -> [x1_(k+1)] = [                 x1_k + dT*G*x2_k                         ]
                                          [x2_(k+1)]   [x2_k + dT*Minv(x1_k,x2_k)*(S'*tau_k+Jc'*fc_k-h(x1_k,x2_k))]
        Linearizing around xc, uc

        x_dot = f(xc, uc) + d f(x,u)/dx |(xc, uc) *(x-xc) + d f(x,u)/du |(xc, uc) *(u-uc)
        x-xc = x_hat,   u-uc=u_hat
        d f1/dx = [d f1/dx1, d f2/dx2]
        d f2/dx = [d f2/dx1, d f2/dx2]

        d f1/dx1 = I,       d f1/dx2 = G*dT
        d f2/dx1 = d/dq [dT*Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dq
        d f2/dx2 = d/dq_dot [dT*Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dv

                   [  I      G*dT  ]
        => df/dx = [               ] = A
                   [ddq_dq  ddq_dv ]

        d f1/du = [d_f1/d_tau, d_f1/d_fc] = [   0,       0    ]
        d f2/du = [d_f2/d_tau, d_f2/d_fc] = [Minv*S', Minv*Jc']

                    [   0               0     ]
        => df/du =  [                         ] = B
                    [dT*Minv*S'    dT*Minv*Jc']

        -> x_dot = A*x_hat + B*u_hat
    */
Task MPC::dynamic_constraint(GeneralizedPosesWithTime gen_poses, Model robot_model, Data data, Eigen::VectorXd q_, Eigen::VectorXd v_){
    
    int q_dim = robot_model.nq;
    int v_dim = robot_model.nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = robot.get_contact_feet_dim()*3;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*(q_dim+v_dim), mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*(q_dim+v_dim));
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    Eigen::VectorXd fc = this->robot.get_optimal_contact_forces()[0];

    Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(q_dim+v_dim, q_dim+v_dim);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(q_dim+v_dim);
    Eigen::VectorXd f_x0 = Eigen::VectorXd::Zero(q_dim+v_dim);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(q_dim+v_dim, joint_dim+contact_forces_dim);
    Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(q_dim+v_dim, q_dim+v_dim);

    auto qw = q(6);
    auto qx = q(3);
    auto qy = q(4);
    auto qz = q(5);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
    Q.row(0) << qw, -qz, qy;
    Q.row(1) << qz, qw, -qx;        // Dynamics of quaternions: quat_dot = 0.5*Q*ang_vel
    Q.row(2) << -qy, qx, qw;
    Q.row(3) << -qx, -qy, -qz;

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(q_dim, v_dim);
    G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3,3);
    G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
    G.block(3, 3, 4, 3) = 0.5*Q;

    robot.compute_terms(q);

    // A0 = [   0      G  ]     =>  A0*x0 = [        G*v0       ]
    //      [ddq_dq ddq_dv]                 [ddq_dq*q0+ddq_dv*v0]
    A0.topRightCorner(q_dim, v_dim) = G;
    pinocchio::computeABADerivatives(robot_model, data, q, v, tau);
    Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd ddq_dq = data.ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

    pinocchio::nonLinearEffects(robot_model, data, q, v);

    A0.bottomLeftCorner(v_dim, q_dim) = ddq_dq;
    A0.bottomRightCorner(v_dim, v_dim) = ddq_dv;

    x0.head(q_dim) = q;
    x0.tail(v_dim) = v;

    this->Minv = data.Minv;

    std::cout <<"Jc transpose: \n" <<Jc_in_time[0].transpose() <<std::endl;
    std::cout <<"M inverse:\n" <<Minv <<std::endl;
    std::cout <<"Optimal contact forces dentro dynamic task:\n" <<fc <<std::endl;
    std::cout <<"Jc.transpose()*fc:\n" <<Jc_in_time[0].transpose() * fc <<std::endl;
    std::cout <<"Non linear effects:\n" <<data.nle <<std::endl;

    f_x0.head(q_dim) = G*v;
    f_x0.tail(v_dim) = Minv*(tau + Jc_in_time[0].transpose()*fc - data.nle);    // M*v_dot + h = tau + Jc*fc

    std::cout <<"f(x0).tail senza termine di forze di contatto:\n" <<data.Minv*(tau - data.nle) <<std::endl;
    std::cout <<"f(x0).tail totale:\n" <<data.Minv*(tau + Jc_in_time[0].transpose()*fc - data.nle) <<std::endl;

    B.bottomLeftCorner(v_dim, joint_dim) = data.Minv.rightCols(joint_dim);
    B.bottomRightCorner(v_dim, contact_forces_dim) = Minv*Jc_in_time[0].transpose();

    A.topLeftCorner(q_dim+v_dim, q_dim+v_dim) = Eigen::MatrixXd::Identity(q_dim+v_dim, q_dim+v_dim);
    A.block(0, mpc_step_horizon*(q_dim+v_dim), q_dim+v_dim, joint_dim+contact_forces_dim) = -B*dT;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(q_dim+v_dim);
    u << Eigen::VectorXd::Zero(q_dim), data.Minv*(tau + Jc_in_time[0].transpose()*fc)*dT;

    b.head(q_dim+v_dim) = x0  + f_x0*dT - u;
    // => [I -B][x_(k+1)] = [x0] + dT*[         G*v         ] - B*uc   -> x_(k+1) = x0 + B*(u_k - u_c) + dT*[         G*v         ]
    //          [ u_(k) ]             [M^(-1)*(tau+Jc'*fc-h)]                                               [M^(-1)*(tau+Jc'*fc-h)]

    for(int i=1; i<mpc_step_horizon; i++){
        B.setZero();
        Q.setZero();
        Ai.setZero();
        q = q_propagated[i];
        v = v_propagated[i];

        qw = q(6);
        qx = q(3);
        qy = q(4);
        qz = q(5);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        robot.compute_terms(q);
        Ai.topLeftCorner(q_dim, q_dim) = Eigen::MatrixXd::Identity(q_dim, q_dim);
        Ai.topRightCorner(q_dim, v_dim) = dT*G;
        pinocchio::computeABADerivatives(robot_model, data, q, v, tau);
        Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd ddq_dq = data.ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
        Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.
        Ai.bottomLeftCorner(v_dim, q_dim) = dT*ddq_dq;
        Ai.bottomRightCorner(v_dim, v_dim) = dT*ddq_dv;

        B.bottomLeftCorner(v_dim, joint_dim) = dT*data.Minv.rightCols(joint_dim);
        B.bottomRightCorner(v_dim, v_dim) = dT*data.Minv*Jc_in_time[i].transpose();

        A.block(i*(q_dim+v_dim), i*(q_dim+v_dim), q_dim+v_dim, q_dim+v_dim) = Eigen::MatrixXd::Identity(q_dim+v_dim, q_dim+v_dim);
        A.block(i*(q_dim+v_dim), (i-1)*(q_dim+v_dim), q_dim+v_dim, q_dim+v_dim) = Ai;
        A.block(i*(q_dim+v_dim), mpc_step_horizon*(q_dim+v_dim)+i*(joint_dim+contact_forces_dim), q_dim+v_dim, joint_dim+contact_forces_dim) = -B;
    }

    Task dynamic_constraints(A, b, D, f);
    //std::cout <<"\nDynamic constraint: \n" <<dynamic_constraints <<"\n" <<std::endl;
    return dynamic_constraints;
}

// *******************************************************************************************************************************************************

// ##################################################################################################################################### 
// #                                                     DYNAMIC CONSTRAINT                                                            #
// #####################################################################################################################################

/*
Task MPC::dynamic_constraint(GeneralizedPosesWithTime gen_poses, Model robot_model, Data data, Eigen::VectorXd q_, Eigen::VectorXd v_){
    /*
        x_dot = f(x, u) -> x_(k+1) = x_k + dT*f(x_k, u_k)

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

        Dynamic: M(q, v)*v_dot + h(q, v) = S'*tau + J_c'*fc

        Relationship between derivative of quaternion and angular velocity:
        quat = q0 + q1*i + q2*j + q3*k,        w = 0 + wx*i + wy*j + wz*k
        quat = [q0, q1, q2, q3] (q0=scalar part), w = [0, wx, wy, wz]

        quat_dot = 0.5 * W (*) quat   (*) = quaternion product

        q0_dot = -0.5*(wx*q1+wy*q2+wz*q3)       [q1_dot]         [ q0 -q3  q2] [wx]
        q1_dot = 0.5*(wx*q0-wy*q3+wz*q2)    =>  [q2_dot] =  0.5* [ q3  q0 -q1] [wy]
        q2_dot = 0.5*(wy*q0-wz*q1+wx*q3)        [q3_dot]         [-q2  q1  q0] [wz]
        q3_dot = 0.5*(wz*q0+wy*q1-wx*q2)        [q0_dot]         [-q1 -q2 -q3]

        
                              [I    0   0] * [      v_b     ]
        q_dot != v => q_dot = [0    Q   0]   [      w_b     ] = G*v
                              [0    0   I]   [ q_dot_joints ]

            [  q  ]   [x1]             [x1_dot]   [q_dot]   [                G*x2                  ]   [f1(x, u)]
        x = [     ] = [  ]  -> x_dot = [      ] = [     ] = [                                      ] = [        ] = f(x, u)
            [  v  ]   [x2]             [x2_dot]   [v_dot]   [ Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2)) ]   [f2(x, u)]

        x_(k+1) = x_k + dT*f(x_k, u_k) -> [x1_(k+1)] = [                 x1_k + dT*G*x2_k                         ]
                                          [x2_(k+1)]   [x2_k + dT*Minv(x1_k,x2_k)*(S'*tau_k+Jc'*fc_k-h(x1_k,x2_k))]
        Linearizing around xc, uc

        x_dot = f(xc, uc) + d f(x,u)/dx |(xc, uc) *(x-xc) + d f(x,u)/du |(xc, uc) *(u-uc)
        x-xc = x_hat,   u-uc=u_hat
        d f1/dx = [d f1/dx1, d f2/dx2]
        d f2/dx = [d f2/dx1, d f2/dx2]

        d f1/dx1 = I,       d f1/dx2 = G*dT
        d f2/dx1 = d/dq [dT*Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dq
        d f2/dx2 = d/dq_dot [dT*Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))] = ddq_dv

                   [  I      G*dT  ]
        => df/dx = [               ] = A
                   [ddq_dq  ddq_dv ]

        d f1/du = [d_f1/d_tau, d_f1/d_fc] = [   0,       0    ]
        d f2/du = [d_f2/d_tau, d_f2/d_fc] = [Minv*S', Minv*Jc']

                    [   0               0     ]
        => df/du =  [                         ] = B
                    [dT*Minv*S'    dT*Minv*Jc']

        -> x_dot = A*x_hat + B*u_hat
        // CHIUDERE IL COMMENTO

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim-6;
    int contact_forces_dim = 3*this->robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int input_dim = joint_dim+contact_forces_dim;

    int dim_A = q_dim + v_dim;
    Eigen::VectorXd q = q_;
    // std::cout <<"Stampa q\n" <<q <<"\n" <<std::endl;;
    Eigen::VectorXd v = v_;
    // std::cout <<"Stampa q_dot\n" <<v <<"\n" <<std::endl;
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    // std::cout <<"Stampa tau\n" <<tau <<"\n" <<std::endl;

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

    Eigen::MatrixXd A0 = MatrixXd::Zero(dim_A, dim_A);
    Eigen::MatrixXd Bi = MatrixXd::Zero(dim_A, input_dim);
    Eigen::MatrixXd S = MatrixXd::Zero(joint_dim, v_dim);
    S.rightCols(joint_dim).setIdentity();

    Eigen::MatrixXd A = MatrixXd::Zero(dim_A*mpc_step_horizon, mpc_step_horizon*(dim_A+joint_dim+contact_forces_dim));
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*dim_A);
    Eigen::MatrixXd D = MatrixXd::Zero(0, mpc_step_horizon*(dim_A+joint_dim+contact_forces_dim));
    Eigen::VectorXd f = VectorXd::Zero(0);

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(q_dim, v_dim);

    // G = [I, 0, 0]                                                     [  p_base_dot ]    [v_base]
    //     [0, Q, 0]        Dynamics of q_dot with respect to v: q_dot = [quat_base_dot] =  [0.5*Q*omega_base]
    //     [0, 0, I]                                                     [q_joints_dot ]    [q_joints_dot]
    G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
    G.block(3, 3, 4, 3) = Q*0.5;
    G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
    // std::cout <<"Matrix G, of dimensions: " <<G.rows() <<" rows, " <<G.cols() <<" cols:\n" <<G <<std::endl;
    // std::cout <<"Matrix G inverse, of dimensions: " <<G.completeOrthogonalDecomposition().pseudoInverse().rows() <<" rows, " <<G.completeOrthogonalDecomposition().pseudoInverse().cols() <<" cols:\n" <<G.completeOrthogonalDecomposition().pseudoInverse() <<std::endl;

    pinocchio::computeABADerivatives(robot_model, data, q, v, tau);
    Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd ddq_dq = data.ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    
    Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.
    // std::cout <<"ddq_dq, of dimensions: " <<ddq_dq.rows() <<" rows, " <<ddq_dq.cols() <<" cols:\n" <<ddq_dq <<"\n" <<std::endl;
    // std::cout <<"ddq_dv, of dimensions: " <<ddq_dv.rows() <<" rows, " <<ddq_dv.cols() <<" cols:\n" <<ddq_dv <<"\n" <<std::endl;

    //ddq_dq.setZero();
    //ddq_dv.setZero();

    // Construction of b vector
    Eigen::VectorXd x0 = VectorXd::Zero(dim_A);
    Eigen::VectorXd f_x0 = Eigen::VectorXd::Zero(dim_A);

    robot.compute_terms(q);
    Eigen::MatrixXd Jc = Jc_in_time[0];
    // std::cout <<"Jc:\n" <<Jc <<std::endl;
 
    pinocchio::nonLinearEffects(robot_model, data, q, v);
    Eigen::VectorXd nle = data.nle;

    Eigen::VectorXd optimal_torque = Eigen::VectorXd::Zero(v_dim);
    optimal_torque.tail(joint_dim) = robot.get_optimal_torques()[0];

    // std::cout <<"Non linear effects:\n" <<nle <<std::endl;
    // std::cout <<"optimal contact force:\n" <<robot.get_optimal_contact_forces()[0] <<std::endl;
    // std::cout <<"Optimal torque:\n" <<optimal_torque <<std::endl;
    // std::cout <<"Minv:\n" <<data.Minv <<std::endl;
    // std::cout <<"Jc transpose:\n" <<Jc.transpose() <<std::endl;

    // COSTRUZIONE MATRICE A0 CON EQUAZIONI LINEARIZZATE
    // A0 = [    I             G*dT      ]  => q_(k+1) = q_k + dT*G*v_k + ...
    //      [ddq_dq*dT,     I + ddq_dv*dT]  => v_(k+1) = dT*(dFD/dq *q_k + dFD/dv *v_k) + v_k + ...
    A0.topLeftCorner(q_dim, q_dim) = Eigen::MatrixXd::Identity(q_dim, q_dim);
    //A0.bottomRightCorner(v_dim, v_dim) = Eigen::MatrixXd::Identity(v_dim, v_dim);
    A0.topRightCorner(q_dim, v_dim) = G*dT;
    A0.bottomLeftCorner(v_dim, q_dim) = ddq_dq*dT;
    A0.bottomRightCorner(v_dim, v_dim) = ddq_dv*dT + Eigen::MatrixXd::Identity(v_dim, v_dim);
    //A0.bottomRightCorner(v_dim, v_dim) = Eigen::MatrixXd::Identity(v_dim, v_dim);
    // std::cout <<"Stampa A0\n" <<A0 <<"\n" <<std::endl;

    x0.head(q_dim) = q;
    x0.tail(v_dim) = v;

    // f(x0) = [          dT*G*v          ]
    //         [dT*Minv*(Jc' fc + tau - h)]
    f_x0.segment(0, q_dim) = dT*G*v + q;
    f_x0.segment(q_dim, v_dim) = dT*data.Minv*(tau-nle) + v;
    //f_x0.segment(q_dim, v_dim) = dT*data.Minv*nle;

    // A0*x0 = [    I             G*dT      ] * [q0] = [q0 + dt*G*v]
    //         [ddq_dq*dT,     I + ddq_dv*dT]   [v0]   [df2/dq *dT*q0 + df2/dv *dT*v0 + v0]
    b.head(dim_A) = A0*x0 + f_x0;
    // std::cout <<"b:\n" <<b <<std::endl;

    // B = [   0            0     ]
    //     [dT*Minv    dT*Minv*Jc'] 
    Bi.bottomRows(v_dim) << dT*data.Minv*S.transpose(), dT*data.Minv*Jc.transpose();
    // std::cout <<"Bi:\n" <<Bi <<std::endl;
    A.block(0, mpc_step_horizon*dim_A, dim_A, input_dim) = -Bi;

    for (int j=0; j<mpc_step_horizon; j++){
        A.block(j * dim_A, j * dim_A, dim_A, dim_A) = MatrixXd::Identity(dim_A, dim_A);
    }

    // A = [I -B],  b = A_0*x_0 + f(x_0) + x_0, x_opt = [x_1; u_0]
    //      => I*x_1 - B_0*u_0 = A_0*x_0 + f(x_0)    -> x_1 = f(x_0) + A_0*x_0 + B_0*u_0

    Eigen::MatrixXd Ai = MatrixXd::Zero(dim_A, dim_A);
    for (int j=0; j<mpc_step_horizon-1; j++){
        Ai.setZero();
        Q.setZero();
        Jc.setZero();
        tau = robot.get_optimal_input()[j];
        q = q_propagated[j+1];
        v = v_propagated[j+1];

        auto qw = q(6);
        auto qx = q(3);
        auto qy = q(4);
        auto qz = q(5);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
        G.block(3, 3, 4, 3) = Q;
        G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
        pinocchio::computeABADerivatives(robot_model, data, q, v, tau);
        Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
        ddq_dq = data.ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
        ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

        Ai.block(0, q_dim, 3, 3) = Eigen::MatrixXd::Identity(3,3);
        Ai.block(3, q_dim+3, 4, 3) = Q;
        Ai.block(7, q_dim+6, joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
        Ai.bottomLeftCorner(v_dim, v_dim) = ddq_dq;
        Ai.bottomRightCorner(v_dim, v_dim) = ddq_dv; 


        A.block(j*dim_A + dim_A, j*dim_A, dim_A, dim_A) = -Ai;
        // std::cout <<"A:\n" <<A <<"\n";

        robot.compute_terms(q);
        Jc = Jc_in_time[j];

        Bi.setZero();
        //Bi.topLeftCorner(v_dim, v_dim) = data.Minv;
        Bi.topLeftCorner(v_dim, joint_dim) = data.Minv * S.transpose();
        std::cout <<"Stampa Bi\n" <<Bi <<"\n";
        Bi.bottomRightCorner(v_dim, contact_forces_dim) = data.Minv*Jc.transpose();
        A.block(j*dim_A + dim_A, j*input_dim + mpc_step_horizon*dim_A, dim_A, input_dim ) = -Bi;
    }

    std::cout <<"Dimensioni matrice A dinamica:\n" <<A.rows() <<"   " <<A.cols() <<"\n";
    Task dynamic_constraints(A, b, D, f);
    // std::cout <<"\nDynamic constraint: \n" <<dynamic_constraints <<"\n" <<std::endl;
    return dynamic_constraints;
}
*/
// ######################################################################################################################################
//                                                    TORQUE LIMITS CONSTRAINT                                                          #
// ######################################################################################################################################

Task MPC::torque_limits_constraint(){
    /*
        tau <= u.b
        state = [q', q_dot', tau', fc']'
        [0 ... 0 I^A 0 ... 0] [     [q_1', qdot_1']'     ]    [u.b]
        [0 ... 0 0 I^A ... 0] [         [...]            ]    [u.b]
        [         ...       [ [ [q_(N-1)', qdot_(N-1)']' ] <= [u.b]         I^A = [I, 0]
        [0 ... 0 0 0 ... I^A] [     [tau_1', fc_1']'     ]    [u.b]               [0, 0]
                              [         [...]            ]    
                              [ [tau_(N-1)', fc_(N-1)']  ]    
    */

    int q_dim = model.nq;
    int v_dim = model.nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (q_dim + v_dim + joint_dim + contact_forces_dim);
    
    Eigen::MatrixXd A = MatrixXd::Zero(0, cols);
    Eigen::VectorXd b = VectorXd::Zero(0);
    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*joint_dim, cols);
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*joint_dim);

    for(int i = 0; i<mpc_step_horizon; i++){
        // std::cout <<"Jc_in_time:\n" <<Jc_in_time[i] <<std::endl;
        D.block(i*joint_dim, (q_dim+v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
        D.block(i*joint_dim+joint_dim, (q_dim+v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim, joint_dim) = -MatrixXd::Identity(joint_dim, joint_dim);
        f.segment(i*2*joint_dim, joint_dim) = VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_max();
        f.segment(i*2*joint_dim + joint_dim, joint_dim) = -VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_min();
    
    }

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
   std::cout <<"Stampa motion tracking\n" <<std::endl;
    //int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(mpc_step_horizon*(2*joint_dim), cols);
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*(2*joint_dim));

    Eigen::MatrixXd D = MatrixXd::Zero(0, cols);
    Eigen::VectorXd f = VectorXd::Zero(0);

    A.leftCols(mpc_step_horizon*(2*joint_dim)) = MatrixXd::Identity(mpc_step_horizon*(2*joint_dim), mpc_step_horizon*(2*joint_dim));

    Eigen::VectorXd joint_des_pos = VectorXd::Zero(robot.get_state_dim());
    Eigen::VectorXd joint_des_pos_prev = VectorXd::Zero(robot.get_state_dim());
    Eigen::VectorXd joint_des_vel = VectorXd::Zero(robot.get_state_dim());

        // #################################################################################################
        joint_des_pos << 0.0, 0.3, -0.6, 0.0, 0.3, -0.6, 0.0, -0.3, 0.6, 0.0, -0.3, 0.6;
        // ################################################################################################

    // std::cout << "A:\n" <<A <<"\n" <<std::endl;
    for (int i=0; i<mpc_step_horizon; i++){
        std::cout <<"i motion track: " <<i <<std::endl;
        //joint_des_pos = robot.clik_alg(gen_poses.generalized_poses_with_time[i].gen_pose, robot.get_robot_model(), robot.get_robot_data());
        // std::cout <<"joint_des_pos:\n" <<joint_des_pos <<"\n" <<std::endl;
        //joint_des_vel = (joint_des_pos-joint_des_pos_prev)/dT;
        // std::cout <<"joint_des_vel:\n" <<joint_des_vel <<"\n" <<std::endl;
        joint_des_pos_prev = joint_des_pos;
        std::cout <<"Posizioni desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos <<"\n";
        std::cout <<"Velocità desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel <<"\n";

        /*
        b.segment(2*i*(joint_dim), joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel;
        */

        b.segment(2*i*(joint_dim), joint_dim) = joint_des_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = joint_des_vel;

    }
    // std::cout <<"b: \n" <<b <<"\n" <<std::endl;

     Task motion_tracking(A, b, D, f);
     return motion_tracking;
    
}

// ################################################################################################################################
// #                                                 MOTION TRACKING OF BASE                                                      #
// ################################################################################################################################

/*
        [vb_dot]   [vb_dot_ref]        [vb_ref - vb]        [  pb_ref - pb   ]
        [      ] = [          ] + Kd * [           ] + Kp * [                ]
        [wb_dot]   [wb_dot_ref]        [wb_ref - wb]        [Q^+ *(qb_ref-qb)]

        vb_dot = (vb_k - vb_(k-1))/dT
*/

Task MPC::motion_tracking_base_constraint(GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){
    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim-6;

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;

    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];

    robot.compute_terms(q);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6*mpc_step_horizon, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6*mpc_step_horizon);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    std::cout <<"Desired base position, velocity and acceleration:\n" <<gen_poses.generalized_poses_with_time[0].gen_pose.base_pos <<"\n\n"
    << gen_poses.generalized_poses_with_time[0].gen_pose.base_vel <<"\n\n" <<gen_poses.generalized_poses_with_time[0].gen_pose.base_acc <<"\n" <<std::endl;
    std::cout <<"Desired base angular velocity and attitude:\n" <<gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel <<"\n\n"
    <<gen_poses.generalized_poses_with_time[0].gen_pose.base_quat <<std::endl;

    //A.block(0, q_dim, 6, v_dim) = Jb;
    A.block(0, q_dim, 6, 6) = Eigen::MatrixXd::Identity(6,6);
    // std::cout <<"Matrice A:\n" <<A <<std::endl;
    
    pinocchio::FrameIndex base_id = 1;
    Eigen::VectorXd base_classical_acc = pinocchio::getClassicalAcceleration(model, data, base_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
    Eigen::Matrix3d kp_pos = Eigen::Matrix3d::Identity(3,3)*Kp_pos;
    Eigen::Matrix3d kd_pos = Eigen::Matrix3d::Identity(3,3)*Kd_pos;
    Eigen::Matrix3d kp_ang = Eigen::Matrix3d::Identity(3,3)*Kp_ang;
    Eigen::Matrix3d kd_ang = Eigen::Matrix3d::Identity(3,3)*Kd_ang;

    std::cout <<"Actual base position, velocity and acceleration:\n" <<q.head(3) <<"\n\n" <<v.head(3) <<"\n\n" <<base_classical_acc.topRows(3) <<"\n" <<std::endl;
    std::cout <<"Actual base orientation and angular velocity:\n" <<q.segment(3, 4) <<"\n\n" <<v.segment(3, 3) <<"\n" <<std::endl;

    b.segment(0, 3) = gen_poses.generalized_poses_with_time[0].gen_pose.base_acc 
        + kd_pos*(gen_poses.generalized_poses_with_time[0].gen_pose.base_vel - v.head(3) )
        + kp_pos*(gen_poses.generalized_poses_with_time[0].gen_pose.base_pos - q.head(3) )
        - base_classical_acc.topRows(3);
    b.segment(0, 3) = b.segment(0, 3).eval()*dT + v.head(3);    // v_dot = (v_(k+1)-v_k)/dT
    // std::cout <<"Stampa b:\n" <<b <<std::endl;
   /*
    Eigen::VectorXd v_prova = Eigen::VectorXd::Zero(3);
    v_prova << 0, 0, 0;
    Eigen::VectorXd p_prova = Eigen::VectorXd::Zero(3);
    p_prova << 0, 0, 0.3;
    b.segment(0, 3) = kd_pos*(v_prova - v.head(3)) + kp_pos*(p_prova-q.head(3));
    b.segment(0, 3) = b.segment(0, 3).eval()*dT + v.head(3);
    */

    Eigen::VectorXd e_a_lin = gen_poses.generalized_poses_with_time[0].gen_pose.base_acc - base_classical_acc.topRows(3);
    Eigen::VectorXd e_v_lin = gen_poses.generalized_poses_with_time[0].gen_pose.base_vel - v.head(3);
    Eigen::VectorXd e_p_lin = gen_poses.generalized_poses_with_time[0].gen_pose.base_pos - q.head(3);
    std::cout <<"Errore su accelerazione lineare:\n" <<e_a_lin <<"\n" <<std::endl;
    std::cout <<"Errore su velocità lineare:\n" <<e_v_lin <<"\n" <<std::endl;
    std::cout <<"Errore su posizione :\n" <<e_p_lin <<"\n" <<std::endl;
    //std::cout <<"Kp_pos: " <<Kp_pos <<std::endl;
    //std::cout <<"Kd_pos: " <<Kd_pos <<std::endl;
    std::cout <<"Componente di errore totale non scalata:\n" <<e_a_lin+e_p_lin+e_v_lin <<"\n" <<std::endl;
    std::cout <<"Componente di errore totale scalata della componente lineare:\n" <<e_a_lin+Kp_pos*e_p_lin+Kd_pos*e_v_lin <<"\n" <<std::endl;
    std::cout <<"Componente di b della componente lineare:\n" <<dT*(e_a_lin+Kp_pos*e_p_lin+Kd_pos*e_v_lin)+v.head(3) <<"\n" <<std::endl;

    // Quaternion:  q = qw*1 + qx*i + qy*j + qz*k
    auto qw = q(6);
    auto qx = q(3);
    auto qy = q(4);
    auto qz = q(5);

    Quaterniond quat = Quaternion<double>(qw, qx, qy, qz);
    Quaterniond quat_des = Quaternion<double>(gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.w(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.x(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.y(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.z());
    //b.segment(3, 3) = kd_ang*(gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel - v.segment(3, 3))
    //                + kp_ang*( Q.completeOrthogonalDecomposition().pseudoInverse()*(gen_poses.generalized_poses_with_time[0].gen_pose.base_quat - q.segment(3,4)) )
    //                - base_classical_acc.bottomRows(3);

    b.segment(3, 3) = kd_ang*(gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel - v.segment(3, 3))
                    + kp_ang*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) )
                    - base_classical_acc.bottomRows(3);
    b.segment(3,3) = b.segment(3,3).eval()*dT + v.segment(3,3);
    std::cout <<"Errore su velocità angolare:\n" <<gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel - v.segment(3, 3) <<std::endl;
    std::cout <<"Errore su orientazione:\n" <<pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) <<std::endl;

    for (int i=1; i<mpc_step_horizon; i++){
        base_classical_acc.setZero();

        tau = robot.get_optimal_input()[i];
        q = q_propagated[i];
        v = v_propagated[i];

        auto qw = q(6);
        auto qx = q(3);
        auto qy = q(4);
        auto qz = q(5);

        Quaterniond quat = Quaternion<double>(qw, qx, qy, qz);
        Quaterniond quat_des = Quaternion<double>(gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.w(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.x(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.y(), gen_poses.generalized_poses_with_time[0].gen_pose.base_quat.z());

        A.block(i*6, q_dim + i*(q_dim+v_dim), 6, 6) = Eigen::MatrixXd::Identity(6,6);
        A.block(i*6, q_dim+(i-1)*(q_dim+v_dim), 6, 6) = -Eigen::MatrixXd::Identity(6,6);
        // std::cout <<"Matrice A:\n" <<A <<std::endl;
        
        pinocchio::FrameIndex base_id = 1;
        base_classical_acc = pinocchio::getClassicalAcceleration(model, data, base_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();

        b.segment(i*6, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_acc 
            + kd_pos*(gen_poses.generalized_poses_with_time[i].gen_pose.base_vel - v.head(3) )
            + kp_pos*(gen_poses.generalized_poses_with_time[i].gen_pose.base_pos - q.head(3))
            - base_classical_acc.topRows(3);
        b.segment(i*6, 3) = b.segment(i*6, 3).eval()*dT;    // v_dot = (v_(k+1)-v_k)/dT
        // std::cout <<"Stampa b:\n" <<b <<std::endl;

        b.segment(3+i*6, 3) = kd_ang*(gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel - v.segment(3, 3))
                    + kp_ang*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) )
                    - base_classical_acc.bottomRows(3);
        b.segment(3+i*6, 3) = b.segment(3+i*6, 3).eval()*dT;
    }

    Task motion_tracking(A, b, D, f);
    std::cout <<"Task motion tracking:\n" <<motion_tracking <<"\n" <<std::endl;
    return motion_tracking;
}

// ################################################################################################################################
// #                                             MOTION TRACKING OF SWING FEET                                                    #
// ################################################################################################################################

Task MPC::motion_tracking_swing_feet_constraint(pinocchio::Model model, pinocchio::Data data, GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){

    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim - 6;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*(3*4), mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*12);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(3*4, model.nv);

    Eigen::MatrixXd kp_s_pos = Eigen::MatrixXd::Identity(3,3)*Kp_s_pos;
    Eigen::MatrixXd kd_s_pos = Eigen::MatrixXd::Identity(3,3)*Kd_s_pos;

    Eigen::VectorXd feet_vel = Eigen::VectorXd::Zero(12);

    if (gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names.size() == 4){
        A.resize(0, NoChange);
        b.resize(0, NoChange);
        Task motion_tracking_swing_feet(A, b, D, f);
        std::cout <<"Tracking swing feet:\n" <<motion_tracking_swing_feet <<"\n" <<std::endl;
        return motion_tracking_swing_feet;
    }

    std::vector<std::string> swing_feet_names;
    std::vector<int> swing_feet_index;
    std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
    std::vector<std::string> robot_feet_names = robot.get_feet_names();

    for (int i=0; i<mpc_step_horizon; i++){
        Js.setZero();
        feet_vel.setZero();
        std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        robot.compute_terms(q_propagated[i]);

        swing_feet_names.resize(4-contact_feet_names.size());
        swing_feet_index.resize(4-contact_feet_names.size());

        int counter = 0;
        for (size_t j= 0; j < 4; j++) {
            if ( std::find(contact_feet_names.begin(), contact_feet_names.end(), robot_short_feet_names[j]) == contact_feet_names.end() ) {
                // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing foot.
                std::cout <<"robot_short_feet_names[i]: " <<robot_short_feet_names[j] <<std::endl;
                std::cout <<"robot_feet_names[j]: " <<robot_feet_names[j] <<std::endl;
                swing_feet_index[counter] = j;
                swing_feet_names[counter] = robot_feet_names[j];
                std::cout <<"swing_foot_name[i]:\n" <<swing_feet_names[counter] <<std::endl;
                counter++;
            }
        }

        std::cout <<"Desired swing foot acceleration:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc <<std::endl;
        std::cout <<"Desired foot position and velocity:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(0, 3) <<"\n\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(0, 3) <<std::endl;

        //pinocchio::computeMinverse(model, data, q_propagated[i]);
        robot.get_Js(Js, swing_feet_names, swing_feet_index);
        //std::cout <<"Js:\n" <<Js <<std::endl;

        feet_vel = Js*v_propagated[i];
        //std::cout <<"feet_vel:\n" <<feet_vel <<std::endl;

        for(int k=0; k<swing_feet_index.size(); k++){
            //std::cout <<"swing_feet_index[i]:\n" <<swing_feet_index[k] <<"   " <<std::endl;
            //std::cout <<"swing_feet_names[swing_feet_index[k]]: " <<swing_feet_names[k] <<std::endl;

            pinocchio::FrameIndex foot_id = model.getFrameId(swing_feet_names[k]);
            std::cout <<"foot_id: " << foot_id <<std::endl;
            Eigen::VectorXd swing_foot_classical_acc = pinocchio::getClassicalAcceleration(robot.get_robot_model(), robot.get_robot_data(), foot_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
            std::cout <<"Actual swing_foot_acc:\n" <<swing_foot_classical_acc <<std::endl;
            Eigen::VectorXd swing_foot_vel = pinocchio::getFrameVelocity(robot.get_robot_model(), robot.get_robot_data(), foot_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
            std::cout <<"Actual swing_foot_vel:\n" <<swing_foot_vel <<std::endl;
            std::cout <<"swing_foot_vel computed with Js:\n" <<feet_vel.segment(3*swing_feet_index[k], 3) <<std::endl;
            Eigen::VectorXd foot_pos = robot.get_robot_data().oMf[foot_id].translation();
            std::cout <<"Actual swing_foot_pos:\n" <<foot_pos <<std::endl;

            b.segment(i*12 + swing_feet_index[k]*3, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc
                + kd_s_pos*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(k*3, 3) - swing_foot_vel.topRows(3))
                + kp_s_pos*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(k*3, 3) - foot_pos.topRows(3));
                - swing_foot_classical_acc.topRows(3);
            //std::cout <<"b:\n" <<b <<std::endl;
            //b.segment(i*12 + swing_feet_index[k]*3, 3) = b.segment(i*12+swing_feet_index[k]*3, 3).eval()*dT + swing_foot_vel.segment(k*3, 3);

            Eigen::VectorXd e_a = gen_poses.generalized_poses_with_time[0].gen_pose.feet_acc - swing_foot_classical_acc.topRows(3);
            Eigen::VectorXd e_v = gen_poses.generalized_poses_with_time[0].gen_pose.feet_vel - swing_foot_vel.topRows(3);
            Eigen::VectorXd e_p = gen_poses.generalized_poses_with_time[0].gen_pose.feet_pos - foot_pos.topRows(3);
            std::cout <<"Errore su accelerazione lineare:\n" <<e_a <<"\n" <<std::endl;
            std::cout <<"Errore su velocità lineare:\n" <<e_v <<"\n" <<std::endl;
            std::cout <<"Errore su posizione :\n" <<e_p <<"\n" <<std::endl;
            //std::cout <<"Kp_pos: " <<Kp_pos <<std::endl;
            //std::cout <<"Kd_pos: " <<Kd_pos <<std::endl;
            std::cout <<"Componente di errore totale non scalata:\n" <<e_a+e_p+e_v <<"\n" <<std::endl;
            std::cout <<"Componente di errore totale scalata della componente lineare:\n" <<e_a+Kp_s_pos*e_p+Kd_s_pos*e_v <<"\n" <<std::endl;
            std::cout <<"Componente di b della componente lineare:\n" <<dT*(e_a+Kp_s_pos*e_p+Kd_s_pos*e_v)+swing_foot_vel.topRows(3) <<"\n" <<std::endl;
        }

        //A.block(i*12, q_dim + i*(q_dim + v_dim), 12, v_dim) = Js;

        /*
            [0 0 Js*Minv Js*Minv*Jc'][q_k+1] = Js*Minv*h + ref  => Js*(Minv*tau_k + Minv*Jc'*fc_k - Minv*h) = ref => Js*v_dot = ref
                                     [v_k+1]
                                     [tau_k]
                                     [fc_k ]
        */
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), 12, joint_dim) = Js*Minv.rightCols(joint_dim);
        Eigen::MatrixXd a1 = Js*Minv;
        Eigen::MatrixXd a2 = a1*Jc_in_time[i].transpose();
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim)+joint_dim+i*(joint_dim+contact_dim), 12, contact_dim) = a2;
        //A.block(i+12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim)+joint_dim, 12, contact_dim) = a2;
        std::cout <<"A: \n" <<A <<std::endl;

        pinocchio::nonLinearEffects(model, data, q_propagated[i], v_propagated[i]);
        Eigen::VectorXd nle = data.nle;

        b = b.eval() + Js*Minv*nle;
        std::cout <<"b:\n" <<b <<std::endl;
        //std::cout <<"b new:\n" <<b.segment(i*12, 12)-Js*nle <<std::endl;
    }
    std::cout <<"A:\n" <<A <<std::endl;
    std::cout <<"b:\n" <<b <<std::endl;
    Task motion_tracking_swing_feet(A, b, D, f);
    std::cout <<"Tracking swing feet:\n" <<motion_tracking_swing_feet <<"\n" <<std::endl;
    std::cout <<"Stampa per vedere se è tutto a posto\n" <<std::endl;
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

    int q_dim = model.nq;
    int v_dim = model.nv;
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
    // std::cout<<"h\n" <<h <<"\n";
    // std::cout<<"l\n" <<l <<"\n";
    // std::cout<<"n\n" <<n <<"\n";

    Eigen::MatrixXd A = MatrixXd::Zero(3*ncp*mpc_step_horizon, mpc_step_horizon*cols);
    Eigen::VectorXd b = VectorXd::Zero(3*ncp*mpc_step_horizon);
    Eigen::MatrixXd D = MatrixXd::Zero(6*ncp*mpc_step_horizon, cols*mpc_step_horizon);
    Eigen::VectorXd f = VectorXd::Zero(6*ncp*mpc_step_horizon);

    Eigen::MatrixXd Ai = MatrixXd::Identity(3*ncp, contact_forces_dim);
    Eigen::MatrixXd Di = MatrixXd::Zero(6*ncp, joint_dim+contact_forces_dim);
    Eigen::VectorXd fi = VectorXd::Zero(6*ncp);

    for (int i = 0; i < mpc_step_horizon; i++) {
        std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        Di.setZero();
        Ai.setIdentity();       // for the swing feet the contact forces must be 0

        // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        for (auto & foot_name : contact_feet_names) {
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
            int index = it - robot_short_feet_names.begin();
            foot_name = robot.get_feet_names()[index];
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

            fi(4 + 6*index) = robot.get_f_max();
            fi(5 + 6*index) = -robot.get_f_min();


           Ai.block(3*index, 3*index, 3, 3) = MatrixXd::Zero(3,3);      // setting the equality constraint for the standng feet to null
        }

        D.block(i*(6*ncp), q_dim+v_dim + i*Di.cols(), 6*ncp, Di.cols()) = Di;
        f.segment(i*(6*ncp), 6*ncp) = fi;

        A.block(i*Ai.rows() , q_dim+v_dim+joint_dim + i*Ai.cols(), Ai.rows(), Ai.cols()) = Ai;
        //std::cout <<"D:\n" <<D <<"\n" <<std::endl;
        //std::cout <<"f:\n" <<f <<"\n" <<std::endl;
        //std::cout <<"A:\n" <<A <<"\n" <<std::endl;
    }

    if (gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names.size() == 4){
        A.resize(0, NoChange);
        b.resize(0, NoChange);
    }

    Task friction(A, b, D, f);
    //std::cout <<"Friction constraint:\n" <<friction <<"\n" <<std::endl;
    return friction;
}

Task MPC::effort_minimization(){
    int q_dim = model.nq;
    int v_dim = model.nv;
    int joint_dim = v_dim-6;
    int contact_dim = 3*robot.get_contact_feet_dim();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*joint_dim, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*joint_dim);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    for (int i=0; i<mpc_step_horizon; i++){
        A.block(i*joint_dim, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
        A.block(i*joint_dim, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim) + joint_dim, contact_dim, contact_dim) = Jc_in_time[i].rightCols(joint_dim).transpose();
    }

    Task effort_minimization(A, b, D, f);
    // std::cout <<"Effort minimization:\n" <<effort_minimization <<std::endl;
    return effort_minimization;
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

        // std::cout <<"Errore:\n" << error <<"\n";

        fit_f(i) = fit_f(i) + error.transpose()*error;
        // std::cout <<"Fit function:\n" <<fit_f <<"\n";

        // Propagate dynamics
        std::vector<Eigen::VectorXd> joint_state_propagated(2);
        Eigen::VectorXd tau = VectorXd::Zero(state_dim);
        for (int j=1; j<mpc_step_horizon; j++){
            tau = Kp[i]*error + Kd[i]*(error-error_prev)/dT + Ki[i]*error_int;
            // std::cout <<"Tau PID\n" <<tau <<"\n";
            joint_state_propagated = robot.compute_dynamics(q, q_dot, tau, dT);
            q = joint_state_propagated[0];
            q_dot = joint_state_propagated[1];

        // Error computation
            error_prev = error;
            error.head(state_dim) = q - gen_poses.generalized_poses_with_time[j].gen_pose.joint_pos;
            error.tail(state_dim) = q_dot - gen_poses.generalized_poses_with_time[j].gen_pose.joint_vel;
            // std::cout <<"Error:\n" <<error <<"\n";

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
            // std::cout <<"Alpha: " <<alpha <<" Beta: " <<beta <<" Delta: " <<delta <<"\n";
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
