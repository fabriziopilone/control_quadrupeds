#include "model_predictive_control/MPC.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/algorithm/aba-derivatives.hpp"

using namespace Eigen;
using namespace pinocchio;

/*
Model Predictive Control based on SOLO12 robot
*/

MPC::MPC(std::string robot_name="solo12", double dT=1.0/400):robot(robot_name){
    this->robot_name = robot_name;
    this->dT = dT;
    this->mpc_step_horizon = 1;
    this->task_request = {"dynamic_contraint", "torque_limits_constraint", "friction_constraint", "motion_tracking"};

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    for (int j=0; j<mpc_step_horizon; j++){
        optimal_input[j] = VectorXd::Zero(2*(this->robot.get_state_dim()-6));
    }
    this->robot.set_optimal_input(optimal_input);
}

VectorXd MPC::solve_MPC(Eigen::VectorXd q, Eigen::VectorXd q_dot, GeneralizedPose gen_pose){

    robot.set_q(q);
    robot.set_qdot(q_dot);
    robot.set_ground_feet_names(gen_pose.contact_feet_names);

    int state_dim = this->robot.get_state_dim();
    int input_dim = state_dim+contact_forces_dim;
    int joint_dim = state_dim - 6;  // First 6 states are the non-actuated floating base pose

    std::vector<Task> task_vec = create_tasks(task_request);

    HO hierarchical_optimization(task_vec, task_vec.size());
    Eigen::VectorXd opt_inp = hierarchical_optimization.solve_ho();

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    for (int i=0; i<mpc_step_horizon; i++){
        Eigen::VectorXd input_i;
        input_i = opt_inp.segment(state_dim*2 + i*(input_dim+2*state_dim), input_dim);
        optimal_input[i] = input_i;
    }

}

// CREATION OF EACH TASK INSIDE THE "task_request" VECTOR

std::vector<Task> MPC::create_tasks(std::vector<std::string> task_request){

    std::vector <Task> task_vec(task_request.size());

    for (long unsigned int i=0; i<=task_request.size() -1; i++){
        std::string task_req = task_request[i];
        switch(resolveOption(task_req)){
            case 0:     // task_req = dynamica_constraint

                task_vec.push_back(dynamic_constraint(robot.get_robot_model(), robot.get_robot_data()));
            break;

            case 1:     // task_req = torque_limits_constraint

                task_vec.push_back(torque_limits_constraint());
            break;

            case 2:     // task_req = friction_constraint

                task_vec.push_back(friction_constraint());
            break;

            case 3:     // task_req = motion_tracking

                task_vec.push_back(friction_constraint());
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

Task MPC::dynamic_constraint(Model robot_model, Data data){
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
    int joint_dim = robot.get_state_dim() - 6;  // First 6 states are the non-actuated floating base pose
    int contact_forces_dim = 3*this->robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int state_dim = this->robot.get_state_dim();
    int input_dim = state_dim+contact_forces_dim;

    //int dim_A = joint_dim*2;
    int dim_A = 2*robot.get_state_dim();
    Eigen::VectorXd q = robot.get_q();
    Eigen::VectorXd q_dot = robot.get_qdot();
    Eigen::VectorXd tau = this->robot.get_optimal_input()[0];
    pinocchio::computeABADerivatives(robot_model, data, q, q_dot, tau);
    Eigen::MatrixXd ddq_dq = data.ddq_dq;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dv = data.ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

    Eigen::MatrixXd A0 = MatrixXd::Zero(dim_A, dim_A);
    Eigen::MatrixXd Bi = MatrixXd::Zero(dim_A, input_dim);
    Eigen::MatrixXd Jc(3*robot.get_contact_feet_dim(), robot.get_contact_feet_dim());
    // Eigen::MatrixXd B(dim_A*mpc_step_horizon, mpc_step_horizon*input_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(dim_A*mpc_step_horizon, dim_A*mpc_step_horizon+joint_dim+robot.get_contact_feet_dim()*3);
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*dim_A);
    Eigen::MatrixXd D = MatrixXd::Zero(0, dim_A*mpc_step_horizon+joint_dim+robot.get_contact_feet_dim()*3);
    Eigen::VectorXd f = VectorXd::Zero(0);

    A0.topRightCorner(joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
    A0.bottomLeftCorner(joint_dim, joint_dim) = ddq_dq;
    A0.bottomRightCorner(joint_dim, joint_dim) = ddq_dv;

    // Construction of b vector
    Eigen::VectorXd x0 = VectorXd::Zero(dim_A);
    x0.head(dim_A) = q;
    x0.tail(dim_A) = q_dot;
    b.head(dim_A) = A0*x0;

    // Insert B0
    Bi.topLeftCorner(state_dim, state_dim) = data.Minv;
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

        robot.get_Jc(Jc);
        Bi.setZero();
        Bi.topLeftCorner(state_dim, state_dim) = data.Minv;
        Bi.bottomRightCorner(state_dim, contact_forces_dim) = data.Minv*Jc.transpose();
        A.block(j*dim_A + dim_A, j*input_dim + mpc_step_horizon*dim_A, dim_A, input_dim ) = -Bi;
    }

    Task dynamic_constraints(A, b, D, f);
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

    int joint_dim = robot.get_state_dim() -6;   // First 6 states are the non-actuated floating base pose
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);
    
    Eigen::MatrixXd A = MatrixXd::Zero(0, cols);
    Eigen::VectorXd b = VectorXd::Zero(0);

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

    f.head(mpc_step_horizon*(joint_dim+contact_forces_dim)) = VectorXd::Ones(mpc_step_horizon*(joint_dim+contact_forces_dim))*robot.get_tau_max();
    f.tail(mpc_step_horizon*(joint_dim+contact_forces_dim)) = VectorXd::Ones(mpc_step_horizon*(joint_dim+contact_forces_dim))*robot.get_tau_min();

    Task torque_limits(A, b, D, f);
    return torque_limits;
    
}

Task MPC::motion_tracking_constraint(){

    /*
        xi = xi^des

        [I 0 ... 0 0 ... 0] [     [q_1', qdot_1']'     ]    [ x_(1)^des ]
        [0 I ... 0 0 ... 0] [         [...]            ]    [ x_(2)^des ]
        [   ...    0 ... 0] [ [q_(N-1)', qdot_(N-1)']' ] =  [    ...    ]
        [0 0 ... I 0 ... 0] [     [tau_1', fc_1']'     ]    [x_(N-1)^des]
                            [         [...]            ]
                            [ [tau_(N-1)', fc_(N-1)']' ]
    */

    int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (2*joint_dim+joint_dim+contact_forces_dim);

    Eigen::MatrixXd A = MatrixXd::Zero(mpc_step_horizon*(2*joint_dim), cols);
    Eigen::VectorXd b = VectorXd::Zero(mpc_step_horizon*(2*joint_dim));

    Eigen::MatrixXd C = MatrixXd::Zero(0, cols);
    Eigen::VectorXd d = VectorXd::Zero(0);

    A.leftCols(mpc_step_horizon*(2*joint_dim)) = MatrixXd::Identity(mpc_step_horizon*(2*joint_dim), mpc_step_horizon*(2*joint_dim));

     //INSERT DESIRED JOINT POSITION AND VELOCITY

     // NON COMPLETE, I NEED THE DESIRED TRAJECTORY FOR ALL THE N STEPS OF OPTIMIZATION

     Task motion_tracking(A, b, C, d);
     return motion_tracking;
    
}

Task MPC::friction_constraint(){
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


    int joint_dim = robot.get_state_dim() - 6;   // First 6 states are the non-actuated floating base pose
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = 2*joint_dim+joint_dim+contact_forces_dim;
    int ncp = robot.get_contact_feet_dim();     // Number of points of contact (legs on the ground)

    Eigen::MatrixXd h = MatrixXd::Zero(robot.get_contact_feet_dim(), contact_forces_dim);
    Eigen::MatrixXd l = MatrixXd::Zero(robot.get_contact_feet_dim(), contact_forces_dim);
    Eigen::MatrixXd n = MatrixXd::Zero(robot.get_contact_feet_dim(), contact_forces_dim);

    for (int i = 0; i < robot.get_contact_feet_dim(); i++) {
        h.block(i, 3*i, 1, 3) << 1, 0, 0;
        l.block(i, 3*i, 1, 3) << 0, 1, 0;
        n.block(i, 3*i, 1, 3) << 0, 0, 1;
    }

    Eigen::MatrixXd A = MatrixXd::Zero(0, mpc_step_horizon*cols);
    Eigen::VectorXd b = VectorXd::Zero(0, mpc_step_horizon*cols);
    Eigen::MatrixXd D = MatrixXd::Zero(6*ncp*mpc_step_horizon, cols*mpc_step_horizon);
    Eigen::VectorXd f = VectorXd::Zero(6*ncp*mpc_step_horizon);


    Eigen::MatrixXd Di = MatrixXd::Zero(6*ncp, joint_dim+contact_forces_dim);
    Di.block(0, joint_dim, ncp, contact_forces_dim) = h-n*mu;
    Di.block(ncp, joint_dim, ncp, contact_forces_dim) = -(h+n*mu);
    Di.block(2*ncp, joint_dim, ncp, contact_forces_dim) = l-n*mu;
    Di.block(3*ncp, joint_dim, ncp, contact_forces_dim) =-(l+n*mu);
    Di.block(4*ncp, joint_dim, ncp, contact_forces_dim) = n;
    Di.block(5*ncp, joint_dim, ncp, contact_forces_dim) = -n;

    Eigen::VectorXd fi = VectorXd::Zero(6*ncp);
    fi.segment(4*ncp, ncp) = VectorXd::Ones(ncp)*robot.get_f_max();
    fi.segment(5*ncp, ncp) = -VectorXd::Ones(ncp)*robot.get_f_min();

    for (int i = 0; i < mpc_step_horizon; i++) {
        D.block(i*(6*ncp), 2*joint_dim + i*Di.cols(), 6*ncp, Di.cols()) = Di;
        f.segment(i*(6*ncp), 6*ncp) = fi;
    }

    Task friction(A, b, D, f);
    return friction;
}