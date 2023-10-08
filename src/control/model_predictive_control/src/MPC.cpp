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
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking"};

    // ***************************************************************
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint"};
    // ****************************************************************
    //this->task_request = {"motion_tracking"};
    
    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    for (int j=0; j<mpc_step_horizon; j++){
        optimal_input[j] = VectorXd::Zero(2*(this->robot.get_state_dim()-6));
    }
    this->robot.set_optimal_input(optimal_input);
}

std::vector<Eigen::VectorXd> MPC::solve_MPC(Eigen::VectorXd q_, Eigen::VectorXd q_dot_, GeneralizedPosesWithTime gen_poses){

    Eigen::VectorXd q = q_;
    Eigen::VectorXd q_dot = q_dot_;
    robot.set_q(q);
    robot.set_qdot(q_dot);
    //robot.set_ground_feet_names(gen_pose.contact_feet_names);

    int state_dim = this->robot.get_state_dim();
    int contact_forces_dim = this->robot.get_contact_feet_dim();
    int input_dim = state_dim+3*contact_forces_dim;
    //int joint_dim = state_dim - 6;  // First 6 states are the non-actuated floating base pose

    Eigen::VectorXd xi_opt = VectorXd::Zero(2*state_dim + input_dim);

    std::vector<Task> task_vec(task_request.size());
    std::cout <<"Dimensione task_vec prima della valorizzazione:\n " <<task_vec.size() <<"\n";
    task_vec = create_tasks(task_request, gen_poses, q, q_dot);
    std::cout <<"Dimensione vettore di richiesta task: \n" <<task_request.size()<<"\n"; 
    std::cout <<"Dimensione vettore dei task:\n" <<task_vec.size() <<"\n";

    HO hierarchical_optimization(task_vec, task_vec.size());
    std::cout <<"Stampa prima della soluzione ottima in MPC\n";

    xi_opt = hierarchical_optimization.solve_ho();

    std::cout << "Soluzione ottima:\n" <<xi_opt <<"\n";

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_torque(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);

    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), state_dim);
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    robot.compute_terms(q);
    robot.get_Jc(Jc, q);

    for (int i=0; i<mpc_step_horizon; i++){
        Eigen::VectorXd tau_i = VectorXd::Zero(state_dim);
        Eigen::VectorXd f_i = VectorXd::Zero(3*contact_forces_dim);

        tau_i = xi_opt.segment(state_dim*2 + i*(2*state_dim+input_dim), state_dim);
        f_i = xi_opt.segment(state_dim*3 + i*(2*state_dim+input_dim), 3*contact_forces_dim);
        optimal_torque[i] = tau_i;
        optimal_contact_forces[i] = f_i;
        optimal_input[i] = tau_i + Jc.transpose()*f_i;

        joint_state_propagated = robot.compute_dynamics(q, q_dot, optimal_input[i], dT);
        q = joint_state_propagated[0];
        q_dot = joint_state_propagated[1];
        robot.compute_terms(q);
        robot.get_Jc(Jc, q);
    }
    
    std::cout <<"******************** DESIRED TASK LIST:********************\n";
    for(int i=0; i<task_request.size(); i++){
        std::cout <<task_request[i] <<"\n";
    }
    std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal torque at step " << i <<" :\n" <<optimal_torque[i] <<"\n";
    }
    std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal contact forces at step " << i <<" :\n" <<optimal_contact_forces[i] <<"\n";
    }
    std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Optimal input to the joints at step " << i <<" :\n" <<optimal_input[i] <<"\n";
    }
    std::cout <<"\n";

    robot.set_optimal_input(optimal_input);
    return optimal_input;
}

// CREATION OF EACH TASK INSIDE THE "task_request" VECTOR

std::vector<Task> MPC::create_tasks(std::vector<std::string> task_request, GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q, Eigen::VectorXd q_dot){

    std::vector <Task> task_vec(0);

    for (long unsigned int i=0; i<=task_request.size() -1; i++){
        std::string task_req = task_request[i];
        std::cout <<"Requested task:\n" <<task_req <<"\n";
        switch(resolveOption(task_req)){
            case 0:{     // task_req = dynamica_constraint

                std::cout <<"Starting dynamic constraint\n";
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, dynamic_constraint(robot.get_robot_model(), robot.get_robot_data(), q, q_dot));
                std::cout <<"Dimensione vettore dei task dopo aver inserito il task dinamica:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 1:{     // task_req = torque_limits_constraint

                std::cout <<"Starting torque limits constraint\n";
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, torque_limits_constraint());
                std::cout <<"Dimensione vettore dei task dopo aver inserito il task coppie:\n" <<task_vec.size() <<"\n";
            }
            break;

            case 2:     // task_req = friction_constraint

                task_vec.push_back(friction_constraint(gen_poses));
            break;

            case 3:     // task_req = motion_tracking

                task_vec.push_back(motion_tracking_constraint(gen_poses));
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
    else return 100;
}

Task MPC::dynamic_constraint(Model robot_model, Data data, Eigen::VectorXd q_, Eigen::VectorXd q_dot_){
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
    */
   std::cout <<"Stampa dynamic constraint\n";
    //int joint_dim = robot.get_state_dim() - 6;  // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*this->robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int state_dim = this->robot.get_state_dim();
    int input_dim = state_dim+contact_forces_dim;

    //int dim_A = joint_dim*2;
    int dim_A = 2*state_dim;
    Eigen::VectorXd q = q_;
    std::cout <<"Stampa q\n" <<q <<"\n";
    Eigen::VectorXd q_dot = q_dot_;
    std::cout <<"Stampa q_dot\n" <<q_dot <<"\n";
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    std::cout <<"Stampa tau\n" <<tau <<"\n";
    pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
    Eigen::MatrixXd ddq_dq = data.ddq_dq;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

    Eigen::MatrixXd A0 = MatrixXd::Zero(dim_A, dim_A);
    Eigen::MatrixXd Bi = MatrixXd::Zero(dim_A, input_dim);
    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), state_dim);
    // Eigen::MatrixXd B(dim_A*mpc_step_horizon, mpc_step_horizon*input_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(dim_A*mpc_step_horizon, mpc_step_horizon*(dim_A+joint_dim+robot.get_contact_feet_dim()*3));
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*dim_A);
    Eigen::MatrixXd D = MatrixXd::Zero(1, mpc_step_horizon*(dim_A+joint_dim+robot.get_contact_feet_dim()*3));
    Eigen::VectorXd f = VectorXd::Zero(1);

    A0.topRightCorner(joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
     //std::cout <<"Stampa A0\n" <<A0 <<"\n";
    A0.bottomLeftCorner(joint_dim, joint_dim) = ddq_dq;
     //std::cout <<"Stampa A0\n" <<A0 <<"\n";
    A0.bottomRightCorner(joint_dim, joint_dim) = ddq_dv;

    // Construction of b vector
    Eigen::VectorXd x0 = VectorXd::Zero(dim_A);
    x0.head(state_dim) = q;
    x0.tail(state_dim) = q_dot;
    b.head(dim_A) = A0*x0;

    // Insert B0
    std::cout <<"Stampa Minv: \n" <<data.Minv <<"\n";
    Bi.topLeftCorner(state_dim, state_dim) = data.Minv;

    robot.compute_terms(q);

    robot.get_Jc(Jc, q);
    std::cout <<"Stampa Jc:\n" <<Jc <<"\n";
    Bi.bottomRightCorner(state_dim, contact_forces_dim) = data.Minv*Jc.transpose();
    A.block(0, mpc_step_horizon*dim_A, dim_A, input_dim) = -Bi;

    for (int j=0; j<mpc_step_horizon; j++){
        A.block(j * dim_A, j * dim_A, dim_A, dim_A) = MatrixXd::Identity(dim_A, dim_A);
    }

    Eigen::MatrixXd Ai = MatrixXd::Zero(dim_A, dim_A);
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    for (int j=0; j<mpc_step_horizon-1; j++){
        Ai.setZero();
        tau = robot.get_optimal_input()[j];
        joint_state_propagated = robot.compute_dynamics(q, q_dot, tau, dT);
        q = joint_state_propagated[0];
        q_dot = joint_state_propagated[1];
        pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
        ddq_dq = data.ddq_dq;
        ddq_dv = data.ddq_dv;

        Ai.topRightCorner(joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
        Ai.bottomLeftCorner(joint_dim, joint_dim) = ddq_dq;
        Ai.bottomRightCorner(joint_dim, joint_dim) = ddq_dv; 


        A.block(j*dim_A + dim_A, j*dim_A + dim_A, dim_A, dim_A) = Ai;
        std::cout <<"A:\n" <<A <<"\n";

        robot.compute_terms(q);
        //pinocchio::computeJointJacobians(robot.get_robot_model(), robot.get_robot_data(), q);

        robot.get_Jc(Jc, q);
        Bi.setZero();
        Bi.topLeftCorner(state_dim, state_dim) = data.Minv;
        //std::cout <<"Stampa Bi\n" <<Bi <<"\n";
        Bi.bottomRightCorner(state_dim, contact_forces_dim) = data.Minv*Jc.transpose();
        A.block(j*dim_A + dim_A, j*input_dim + mpc_step_horizon*dim_A, dim_A, input_dim ) = -Bi;
    }

    std::cout <<"Dimensioni matrice A dinamica:\n" <<A.rows() <<"   " <<A.cols() <<"\n";
    Task dynamic_constraints(A, b, D, f);
    std::cout <<"Dynamic constraint: \n" <<dynamic_constraints <<"\n";
    return dynamic_constraints;
}

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

   std::cout <<"****************************************************\nTorque limits\n****************************************************\n";
    //int joint_dim = robot.get_state_dim() -6;   // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);
    
    Eigen::MatrixXd A = MatrixXd::Zero(1, cols);
    Eigen::VectorXd b = VectorXd::Zero(1);

/*
    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*(joint_dim+contact_forces_dim), cols );
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*(joint_dim+contact_forces_dim));

    Eigen::MatrixXd I_aug = MatrixXd::Zero(joint_dim+contact_forces_dim, joint_dim+contact_forces_dim);
    I_aug.topLeftCorner(joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);

    for (int i = 0; i < mpc_step_horizon; ++i)
    {
        D.block(i * I_aug.rows(), i * I_aug.cols(), I_aug.rows(), I_aug.cols()) = I_aug;
    }

    Eigen::MatrixXd D_lb(mpc_step_horizon*(joint_dim+contact_forces_dim), cols);
    for (int i = 0; i < mpc_step_horizon; ++i)
    {
        D.block(i * I_aug.rows(), i * I_aug.cols(), I_aug.rows(), I_aug.cols()) = -I_aug;
    }
    D.bottomRightCorner(mpc_step_horizon*(joint_dim+contact_forces_dim), cols) = D_lb;
    */

    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*joint_dim, cols);
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*joint_dim);

    for(int i = 0; i<mpc_step_horizon; i++){
        D.block(i*joint_dim, 2*joint_dim + i*cols, joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
    }
    for(int i=0; i<mpc_step_horizon; i++){
        D.block(i*joint_dim, 2*joint_dim + i*cols, joint_dim, joint_dim) = -MatrixXd::Identity(joint_dim, joint_dim);
    }

    f.head(mpc_step_horizon*(joint_dim+contact_forces_dim)) = VectorXd::Ones(mpc_step_horizon*(joint_dim+contact_forces_dim))*robot.get_tau_max();
    f.tail(mpc_step_horizon*(joint_dim+contact_forces_dim)) = VectorXd::Ones(mpc_step_horizon*(joint_dim+contact_forces_dim))*robot.get_tau_min();

    Task torque_limits(A, b, D, f);
    std::cout <<"Task torque constraint\n" <<torque_limits <<"\n";
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
   std::cout <<"Stampa motion tracking\n";
    //int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(mpc_step_horizon*(2*joint_dim), cols);
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*(2*joint_dim));

    Eigen::MatrixXd D = MatrixXd::Zero(1, cols);
    Eigen::VectorXd f = VectorXd::Zero(1);

    A.leftCols(mpc_step_horizon*(2*joint_dim)) = MatrixXd::Identity(mpc_step_horizon*(2*joint_dim), mpc_step_horizon*(2*joint_dim));

    std::cout << "A:\n" <<A <<"\n";
    for (int i=0; i<mpc_step_horizon; i++){
        std::cout <<"Posizioni desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos <<"\n";
        std::cout <<"Velocità desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel <<"\n";

        b.segment(2*i*(joint_dim), joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel;

    }
    std::cout <<"b: \n" <<b <<"\n";
     //INSERT DESIRED JOINT POSITION AND VELOCITY

     // NON COMPLETE, I NEED THE DESIRED TRAJECTORY FOR ALL THE N STEPS OF OPTIMIZATION

     Task motion_tracking(A, b, D, f);
     return motion_tracking;
    
}

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

   std::cout <<"Stampa friction constraint\n";
    //int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int joint_dim = robot.get_state_dim();
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = 2*joint_dim+joint_dim+contact_forces_dim;
    int ncp = robot.get_contact_feet_dim();     // Number of points of contact (legs on the ground)

    Eigen::MatrixXd h = MatrixXd::Zero(ncp, contact_forces_dim);
    Eigen::MatrixXd l = MatrixXd::Zero(ncp, contact_forces_dim);
    Eigen::MatrixXd n = MatrixXd::Zero(ncp, contact_forces_dim);

    for (int i = 0; i < ncp; i++) {
        h.block(i, 3*i, 1, 3) << 1, 0, 0;
        l.block(i, 3*i, 1, 3) << 0, 1, 0;
        n.block(i, 3*i, 1, 3) << 0, 0, 1;
    }
    std::cout<<"h\n" <<h <<"\n";
    std::cout<<"l\n" <<l <<"\n";
    std::cout<<"n\n" <<n <<"\n";

    Eigen::MatrixXd A = MatrixXd::Zero(3*ncp*mpc_step_horizon, mpc_step_horizon*cols);
    Eigen::VectorXd b = VectorXd::Zero(3*ncp*mpc_step_horizon);
    Eigen::MatrixXd D = MatrixXd::Zero(6*ncp*mpc_step_horizon, cols*mpc_step_horizon);
    Eigen::VectorXd f = VectorXd::Zero(6*ncp*mpc_step_horizon);

    Eigen::MatrixXd Ai = MatrixXd::Identity(3*ncp, contact_forces_dim);
    Eigen::MatrixXd Di = MatrixXd::Zero(6*ncp, joint_dim+contact_forces_dim);
    Eigen::VectorXd fi = VectorXd::Zero(6*ncp);
    std::cout <<"Stampa prima di fi\n";
    fi.segment(4*ncp, ncp) = VectorXd::Ones(ncp)*robot.get_f_max();
    fi.segment(5*ncp, ncp) = -VectorXd::Ones(ncp)*robot.get_f_min();
    std::cout <<"fi:\n" <<fi <<"\n";

    for (int i = 0; i < mpc_step_horizon; i++) {
        GeneralizedPose gen_pose = gen_poses.generalized_poses_with_time[i].gen_pose;
        std::vector<std::string> contact_feet_names = gen_pose.contact_feet_names;
        Di.setZero();

        for (int j=0; j<static_cast<int>(contact_feet_names.size()); j++) {
            std::cout <<"contact_feet_names[j]: " <<contact_feet_names[j] <<"\n";
            std::vector<std::string> robot_feet_names = robot.get_feet_names();
            for (int k=0; k<robot_feet_names.size(); k++){
            std::cout <<"robot_feet_names:\n" <<robot_feet_names[k] <<"\n";}

            auto it = std::find(robot_feet_names.begin(), robot_feet_names.end(), contact_feet_names[j]);
            if (it != robot_feet_names.end()){std::cout <<"Trovato\n";};
        // find returns an iterator to the first element in range [first, last) that compares equal to val, if no element is found, returns last
            int index = std::distance(robot_feet_names.begin(), it);
            std::cout <<"index:\n" <<index <<"\n";

            // For each leg in contact (index) populate the Di matrix, the swing legs will have all zeros
            Di.block(index*6, joint_dim, 1, contact_forces_dim) = h.row(index)-n.row(index)*mu;
            Di.block(1+index*6, joint_dim, 1, contact_forces_dim) = -(h.row(index)+n.row(index)*mu);
            Di.block(2+index*6, joint_dim, 1, contact_forces_dim) = l.row(index)-n.row(index)*mu;
            Di.block(3+index*6, joint_dim, 1, contact_forces_dim) =-(l.row(index)+n.row(index)*mu);
            Di.block(4+index*6, joint_dim, 1, contact_forces_dim) = n.row(index);
            Di.block(5+index*6, joint_dim, 1, contact_forces_dim) = -n.row(index);
            std::cout <<"Di:\n" <<Di <<"\n";

            /*
            Di.block(index , Di.cols()) = MatrixXd::Zero(1, Di.cols());
            Di.block(index + ncp , Di.cols()) = MatrixXd::Zero(1, Di.cols());
            Di.block(index + 2*ncp, Di.cols()) = MatrixXd::Zero(1, Di.cols());
            Di.block(index + 3*ncp, Di.cols()) = MatrixXd::Zero(1, Di.cols());
            Di.block(index + 4*ncp, Di.cols()) = MatrixXd::Zero(1, Di.cols());
            Di.block(index + 5*ncp, Di.cols()) = MatrixXd::Zero(1, Di.cols());
            */

           Ai.block(3*index, 3*index, 3, 3) = MatrixXd::Zero(3,3);
        }

        D.block(i*(6*ncp), 2*joint_dim + i*Di.cols(), 6*ncp, Di.cols()) = Di;
        f.segment(i*(6*ncp), 6*ncp) = fi;

        A.block(i*Ai.rows() , 3*joint_dim + i*Ai.cols(), Ai.rows(), Ai.cols()) = Ai;
        std::cout <<"D:\n" <<D <<"\n";
        std::cout <<"f:\n" <<f <<"\n";
        std::cout <<"A:\n" <<A <<"\n";
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
            std::cout <<"Errore precedente\n" <<error_prev <<"\n";
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
    std::vector<Eigen::VectorXd> pid_gains;
    pid_gains[0] = Kp[alpha]; 
    pid_gains[1] = Kd[alpha];
    pid_gains[2] = Ki[alpha];

    return pid_gains;
}
