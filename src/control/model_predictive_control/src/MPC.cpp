#include "model_predictive_control/MPC.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

using namespace Eigen;
using namespace pinocchio;

/*
Model Predictive Control based on SOLO12 robot
*/

MPC::MPC(std::string robot_name="solo12", double dT=1.0/400, int mpc_step_horizon=1):robot(robot_name){
    this->robot_name = robot_name;
    this->dT = dT;
    this->mpc_step_horizon = mpc_step_horizon;
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet", "effort_minimization"};

    // ***************************************************************
    this->task_request = {"torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet"};
    this->task_request = {"dynamic_constraint", "contact_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_feet", "motion_tracking_base"};
    this->task_request = {"dynamic_constraint", "torque_limits_constraint", "friction_constraint", "motion_tracking_base", "motion_tracking_feet"};
    this->task_request = {"dynamic_constraint", "contact_constraint", "torque_limits_constraint", "friction_constraint"};
    // ****************************************************************
    
    std::vector<Eigen::VectorXd> optimal_torques(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);
    std::vector<Eigen::VectorXd> q_prop(mpc_step_horizon);
    std::vector<Eigen::VectorXd> v_prop(mpc_step_horizon);
    this->Jc_in_time.resize(0);
    std::vector<Eigen::MatrixXd> Jc_dot_time(mpc_step_horizon);
    for (int j=0; j<mpc_step_horizon; j++){
        q_prop[j] = VectorXd::Zero(this->robot.get_robot_model().nq);
        v_prop[j] = VectorXd::Zero(this->robot.get_robot_model().nv);
        Jc_dot_time[j] = Eigen::MatrixXd::Zero(3*this->robot.get_contact_feet_dim(), this->robot.get_state_dim());
        optimal_torques[j] = VectorXd::Zero(this->robot.get_state_dim()-6);
        optimal_contact_forces[j] = VectorXd::Zero(3*this->robot.get_contact_feet_dim());
    }
    this->q_propagated = q_prop;
    this->v_propagated = v_prop;
    this->Jc_dot_in_time = Jc_dot_time;
    this->robot.set_optimal_torques(optimal_torques);
    this->robot.set_optimal_contact_forces(optimal_contact_forces);
    
}

void MPC::compute_all(Eigen::VectorXd q_, Eigen::VectorXd v_, GeneralizedPosesWithTime gen_poses){
    // Computing all the terms
    int q_dim = robot.get_state_dim()+1;
    int v_dim = robot.get_state_dim();
    int joint_dim = v_dim-6;

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(v.size());
    tau.tail(joint_dim) = this->robot.get_optimal_torques()[0];
    Eigen::VectorXd fc = this->robot.get_optimal_contact_forces()[0];
    std::vector<Eigen::VectorXd> propagated_state(2);

    Eigen::MatrixXd Jc = MatrixXd::Zero(3*robot.get_contact_feet_dim(), robot.get_state_dim());
    Eigen::MatrixXd Jc_dot = Eigen::MatrixXd::Zero(3*robot.get_contact_feet_dim(), robot.get_state_dim());
    std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
    std::vector<std::string> robot_feet_names = robot.get_feet_names();
    Jc_in_time.resize(0);
    Jc_dot_in_time.resize(mpc_step_horizon);

    for (int i=0; i<mpc_step_horizon; i++){
        //if(i==0 || this->flag==false){
        if(i==0){
            this->q_propagated[i] = q;
            //std::cout <<"q in compute all:\n" <<q_propagated[i] <<std::endl;
            this->v_propagated[i] = v;

            std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
            std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
            std::vector<int> contact_feet_index(0);
            for (auto & foot_name : contact_feet_names) {
                auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
                int index = it - robot_short_feet_names.begin();
                contact_feet_index.push_back(index);
                foot_name = robot.get_feet_names()[index];
                //std::cout <<"foot_name: " <<foot_name <<std::endl;
            }

            robot.compute_terms(q_propagated[i], v_propagated[i]);
            robot.compute_second_order_FK(q_propagated[i], v_propagated[i]);

            Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(12, v_dim);
            Eigen::MatrixXd Jc_dot = Eigen::MatrixXd::Zero(12, v_dim);
            Eigen::VectorXd Jc_dot_times_v = Eigen::VectorXd::Zero(12);
            robot.get_Jc(Jc, contact_feet_names, contact_feet_index);
            robot.get_Jc_dot(Jc_dot, contact_feet_names, contact_feet_index);
            robot.get_Jc_dot_times_v(Jc_dot_times_v, contact_feet_names, contact_feet_index);
            Jc_in_time.push_back(Jc);
            //std::cout <<"Jc_in_time[0]:\n" <<Jc_in_time[0] <<std::endl;
            Jc_dot_in_time[i] = Jc_dot;
            //std::cout <<"Jc_dot_times_v:\n" <<Jc_dot_times_v <<std::endl;
            //std::cout <<"Jc_dot:\n" <<Jc_dot <<std::endl;
        }
        else{
            std::cout <<i <<std::endl;
            Jc.setZero();
            Jc_dot.setZero();
            robot.compute_terms(q, v);
            robot.compute_second_order_FK(q, v);

            std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
            std::vector<int> contact_feet_index(0);
            for (auto & foot_name : contact_feet_names) {
                auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
                int index = it - robot_short_feet_names.begin();
                contact_feet_index.push_back(index);
                foot_name = robot.get_feet_names()[index];
            }
            robot.get_Jc(Jc, contact_feet_names, contact_feet_index);
            robot.get_Jc_dot(Jc_dot, contact_feet_names, contact_feet_index);
            Jc_in_time.push_back(Jc);
            Jc_dot_in_time[i] = Jc_dot;

            int previous_step_length = robot.get_optimal_torques().size();
            //std::cout <<"Previous step length: " <<previous_step_length <<std::endl;

            if(robot.get_optimal_q().size() == 0){
                for(int p=0; p<mpc_step_horizon; p++){
                    robot.add_optimal_q(q);
                    robot.add_optimal_v(v);
                }
            }

                // Copy last optimal torques to extend the vector
                while(mpc_step_horizon > robot.get_optimal_torques().size()){
                    robot.add_optimal_torque(robot.get_optimal_torques()[previous_step_length-1]);
                    robot.add_optimal_contact_force(robot.get_optimal_contact_forces()[previous_step_length-1]);
                    robot.add_optimal_q(robot.get_optimal_q()[previous_step_length-1]);
                    robot.add_optimal_v(robot.get_optimal_v()[previous_step_length-1]);
                }

                /*
                for(int j=0; j<mpc_step_horizon; j++){
                    std::cout <<"optimal torque[ " <<j <<" ]:\n" <<robot.get_optimal_torques()[j] <<std::endl;
                }
                for(int j=0; j<mpc_step_horizon; j++){
                    std::cout <<"optimal contact force[ " <<j <<" ]:\n" <<robot.get_optimal_contact_forces()[j] <<std::endl;
                }
                */
                tau.tail(joint_dim) = robot.get_optimal_torques()[i];
                fc = robot.get_optimal_contact_forces()[i];

            propagated_state = robot.compute_dynamics(q, v, tau, Jc.transpose()*fc, dT);
            q = propagated_state[0];
            v = propagated_state[1];

            //std::cout <<"q propagated:\n" <<q <<std::endl;
            //std::cout <<"v propagated:\n" <<v <<std::endl;
            this->q_propagated[i] = q;
            this->v_propagated[i] = v;
        }
    }
    //std::cout <<"Ending compute all" <<std::endl;
}

std::vector<Eigen::VectorXd> MPC::solve_MPC(Eigen::VectorXd q_, Eigen::VectorXd v_, GeneralizedPosesWithTime gen_poses){

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = this->robot.get_contact_feet_dim()*3;

    //robot.compute_terms(q, v);
    compute_all(q, v, gen_poses);
    Eigen::VectorXd xi_opt = VectorXd::Zero((q_dim + v_dim + joint_dim + contact_forces_dim)*mpc_step_horizon);

    std::vector<Task> task_vec(task_request.size());
    task_vec = create_tasks(task_request, gen_poses, q, v);


    HO hierarchical_optimization(task_vec, task_vec.size());

    xi_opt = hierarchical_optimization.solve_ho(task_request);

    std::vector<Eigen::VectorXd> optimal_input(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_torques(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_contact_forces(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_q(mpc_step_horizon);
    std::vector<Eigen::VectorXd> optimal_v(mpc_step_horizon);
    Eigen::VectorXd tau_i = VectorXd::Zero(joint_dim);
    Eigen::VectorXd f_i = VectorXd::Zero(contact_forces_dim);
    Eigen::VectorXd q_opt = Eigen::VectorXd::Zero(q_dim);
    Eigen::VectorXd v_opt = Eigen::VectorXd::Zero(v_dim);

    tau_i = xi_opt.segment((q_dim + v_dim)*mpc_step_horizon, joint_dim);
    f_i = xi_opt.segment((q_dim+v_dim)*mpc_step_horizon+joint_dim, contact_forces_dim);
    q_opt = xi_opt.segment(0, q_dim);
    v_opt = xi_opt.segment(q_dim, v_dim);
    optimal_torques[0] = tau_i;
    optimal_contact_forces[0] = f_i;
    optimal_q[0] = q_opt;
    optimal_v[0] = v_opt;

    Eigen::VectorXd optimal_input_ext = Eigen::VectorXd::Zero(robot.get_robot_model().nv); 
    optimal_input_ext.tail(joint_dim) = optimal_torques[0];
    optimal_input[0] = optimal_input_ext;

    for (int i=1; i<mpc_step_horizon; i++){
        tau_i.setZero();
        f_i.setZero();
        q_opt.setZero();
        v_opt.setZero();

        tau_i = xi_opt.segment((q_dim + v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim);
        f_i = xi_opt.segment((q_dim+v_dim)*mpc_step_horizon + joint_dim + i*(joint_dim+contact_forces_dim), contact_forces_dim);
        q_opt = xi_opt.segment(i*(q_dim+v_dim), q_dim);
        v_opt = xi_opt.segment(q_dim+i*(q_dim+v_dim), v_dim);
        optimal_torques[i] = tau_i;
        optimal_contact_forces[i] = f_i;
        optimal_q[i] = q_opt;
        optimal_v[i] = v_opt;
    }
    
    // std::cout <<"******************** DESIRED TASK LIST:********************\n";
    for(int i=0; i<task_request.size(); i++){
        // std::cout <<task_request[i] <<"\n";
    }
    
    // std::cout <<"\n";
    
    for(int i=0; i<mpc_step_horizon; i++){
        //std::cout <<"Optimal torque at MPC step " << i <<" :\n" <<optimal_torques[i] <<"\n" <<std::endl;
    }
    // std::cout <<"\n";
    for(int i=0; i<mpc_step_horizon; i++){
        //std::cout <<"Optimal contact forces at MPC step " << i <<" :\n" <<optimal_contact_forces[i] <<"\n" <<std::endl;
    }
    
    for(int i=0; i<mpc_step_horizon; i++){
        //std::cout <<"Optimal q at step " << i <<" :\n" <<optimal_q[i] <<"\n" <<std::endl;
    }
    for(int i=0; i<mpc_step_horizon; i++){
        //std::cout <<"Optimal v at step " << i <<" :\n" <<optimal_v[i] <<"\n" <<std::endl;
    }
    
   //std::cout <<"**************************************************************************************************\n" <<std::endl;

    robot.set_optimal_torques(optimal_torques);
    robot.set_optimal_contact_forces(optimal_contact_forces);
    robot.set_optimal_q(optimal_q);
    robot.set_optimal_v(optimal_v);

    return optimal_torques;
}

// CREATION OF EACH TASK INSIDE THE "task_request" VECTOR

std::vector<Task> MPC::create_tasks(std::vector<std::string> task_request, GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){

    std::vector <Task> task_vec(0);

    for (long unsigned int i=0; i<=task_request.size() -1; i++){
        std::string task_req = task_request[i];

        switch(resolveOption(task_req)){
            case 0:{     // task_req = dynamic_constraint

                //std::cout <<"\n******************** Starting dynamic constraint ********************\n" <<std::endl;
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, dynamic_constraint(gen_poses, q_, v_));
            }
            break;

            case 1:{     // task_req = torque_limits_constraint

                //std::cout <<"\n******************** Starting torque limits constraint ********************\n" <<std::endl;
                std::vector<Task>::iterator it = task_vec.end();
                task_vec.insert(it, torque_limits_constraint());
            }
            break;

            case 2:{     // task_req = friction_constraint
                //std::cout <<"\n******************** Starting friction constraint ********************\n" <<std::endl;
                task_vec.push_back(friction_constraint(gen_poses));
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
                //std::cout <<"\n******************** Starting base motion tracking constraint ********************\n" <<std::endl;
                task_vec.push_back(motion_tracking_base_constraint(gen_poses, q_, v_));
            }
            break;

            case 5:{    // task_req = motion_tracking_swing_feet
                //std::cout <<"\n******************** Starting feet motion tracking constraint ********************\n" <<std::endl;
                task_vec.push_back(motion_tracking_swing_feet_constraint(gen_poses, q_, v_));
            }
            break;

            case 6:{    // task_req = effort_minimization
                // std::cout << "\n******************** Starting effort minimization ********************\n" <<std::endl;
                task_vec.push_back(effort_minimization());
            }
            break;

            case 8:{    // task_req = contact_constraint
                task_vec.push_back(contact_constraint(gen_poses));
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
    else if(task_name == "contact_constraint") return 8;
    else return 100;
}


Task MPC::contact_constraint(GeneralizedPosesWithTime gen_poses){
    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim-6;
    int contact_forces_dim = 3*robot.get_contact_feet_dim();

    //std::cout <<"mpc_step_horizon dentro contact constr: " <<mpc_step_horizon <<std::endl;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12*mpc_step_horizon, (q_dim+v_dim+joint_dim+contact_forces_dim)*mpc_step_horizon);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(12*mpc_step_horizon);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, (q_dim+v_dim+joint_dim+contact_forces_dim)*mpc_step_horizon);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
    std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names;
    std::vector<int> contact_feet_index(0);
        for (auto & foot_name : contact_feet_names) {
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
            int index = it - robot_short_feet_names.begin();
            contact_feet_index.push_back(index);
            foot_name = robot.get_feet_names()[index];
        }
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(12, v_dim);
    Eigen::MatrixXd Jc_dot = Eigen::MatrixXd::Zero(12, v_dim);
    Eigen::VectorXd Jc_dot_times_v = Eigen::VectorXd::Zero(12);

    robot.compute_terms(q_propagated[0], v_propagated[0]);
    robot.compute_second_order_FK(q_propagated[0], v_propagated[0]);
    robot.get_Jc(Jc, contact_feet_names, contact_feet_index);
    robot.get_Jc_dot(Jc_dot, contact_feet_names, contact_feet_index);
    robot.get_Jc_dot_times_v(Jc_dot_times_v, contact_feet_names, contact_feet_index);

    /*
        Jc*v_dot = -Jc_dot * v  => v_dot = Minv*(tau + Jc'*fc - h)
        [0 0 Jc*Minv Jc*Minv*Jc'] [ q_1 ] = Jc*Minv*h - J_dot*v     ==> Jc*Minv*(tau_0+Jc'*fc_0-h) = -J_dot*v
                                  [ v_1 ]
                                  [tau_0]
                                  [ fc_0]

    */
    
    A.block(0, mpc_step_horizon*(q_dim+v_dim), 12, joint_dim) = Jc_in_time[0]*robot.get_robot_data().Minv.rightCols(joint_dim);     // Jc*Minv * tau0
    A.block(0, mpc_step_horizon*(q_dim+v_dim)+joint_dim, 12, contact_forces_dim) = Jc_in_time[0]*robot.get_robot_data().Minv*(Jc_in_time[0].transpose());   // Jc*Minv*Jc' *fc0
    b.segment(0, 12) = Jc_in_time[0]*robot.get_robot_data().Minv*robot.get_robot_data().nle - Jc_dot_times_v;

    for (int i=1; i<mpc_step_horizon; i++){
        Jc.setZero();
        Jc_dot_times_v.setZero();
        contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;
        contact_feet_index.resize(0);
        for (auto & foot_name : contact_feet_names) {
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
            int index = it - robot_short_feet_names.begin();
            contact_feet_index.push_back(index);
            foot_name = robot.get_feet_names()[index];
        }
        robot.compute_terms(q_propagated[i], v_propagated[i]);
        robot.compute_second_order_FK(q_propagated[i], v_propagated[i]);
        robot.get_Jc_dot_times_v(Jc_dot_times_v, contact_feet_names, contact_feet_index);


        
        /*
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_forces_dim), 12, joint_dim) = Jc_in_time[i]*robot.get_robot_data().Minv.rightCols(joint_dim);
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim)+joint_dim+i*(joint_dim+contact_forces_dim), 12, contact_forces_dim) = Jc_in_time[i]*robot.get_robot_data().Minv*Jc_in_time[i].transpose();

        b.segment(i*12, 12) = Jc_in_time[i]*robot.get_robot_data().Minv*robot.get_robot_data().nle - Jc_dot_times_v;
        */

        // #################################################################################################################

        /*
            Jc*v_dot + Jc_dot * v = 0 => Jc*Minv*(tau_0+Jc'*fc_0-h) + J_dot*v = 0 => f(x,u) = 0
            A = df/dx = [0 Jc_dot],     B = df/du = [Jc*Minv    Jc*Minv*Jc']
            f(x,u) ?= f(xc,uc) + df/dx |(xc,uc) *(x-xc) + df/du|(xc, uc) * (u-uc)

            [0   0   B0   0] [x1]   [...]
            [A1  0   0   B1] [u0] = [A*xc + B*uc - f(xc, uc)]
                             [x2]
                             [u1]
        
       
        robot.get_Jc_dot(Jc_dot, contact_feet_names, contact_feet_index);
        std::cout <<"Jc_dot\n" <<Jc_dot <<std::endl;

        // df/dx = [0_(12x19)   Jc_dot]
        Eigen::MatrixXd df_dx = Eigen::MatrixXd::Zero(12, q_dim+v_dim);
        df_dx.block(0, q_dim, 12, v_dim) = Jc_dot;

        // df/du = [Jc*Minv    Jc*Minv*Jc']
        Eigen::MatrixXd df_du = Eigen::MatrixXd::Zero(12, joint_dim+contact_forces_dim);
        df_du.leftCols(joint_dim) = Jc_in_time[i]*robot.get_robot_data().Minv.rightCols(joint_dim);
        df_du.rightCols(contact_forces_dim) = Jc_in_time[i]*robot.get_robot_data().Minv*(Jc_in_time[i].transpose());

        A.block(i*12, (i-1)*(q_dim+v_dim), 12, q_dim+v_dim) = df_dx;
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim)+i*(joint_dim+contact_forces_dim), 12, joint_dim+contact_forces_dim) = df_du;

        Eigen::VectorXd xc = Eigen::VectorXd::Zero(q_dim+v_dim);
        //xc.head(q_dim) = q_propagated[i];
        //xc.tail(v_dim) = v_propagated[i];
        xc.head(q_dim) = q_propagated[i];
        xc.tail(v_dim) = v_propagated[i];
        Eigen::VectorXd uc = Eigen::VectorXd::Zero(joint_dim+contact_forces_dim);
        uc.head(joint_dim) = robot.get_optimal_torques()[i];
        uc.tail(contact_forces_dim) = robot.get_optimal_contact_forces()[i];

        // f(xc, uc) = Jc*Minv*(tau_c + Jc'*fc_c - h) + Jc_dot*vc
        Eigen::VectorXd f_xc = Eigen::VectorXd::Zero(12);
        f_xc = Jc_in_time[i]*robot.get_robot_data().Minv.rightCols(joint_dim)*robot.get_optimal_torques()[i] + Jc_in_time[i]*robot.get_robot_data().Minv*Jc_in_time[i].transpose()*robot.get_optimal_contact_forces()[i] - Jc_in_time[i]*robot.get_robot_data().Minv*robot.get_robot_data().nle + Jc_dot*v_propagated[i];

        b.segment(i*12, 12) = df_dx*xc + df_du*uc - f_xc;       // df_dx*x_k + df_du*u_k = -f(xc, uc) + df_dx*xc + df_du*uc => f(xc, uc) + df_dx*(x_k-xc) + df_du*(u_k-uc) = 0
        */

        // #################################################################################################################

        // #################################################################################################################

        /*
            vc_dot = 0 => Jc*v_dot + Jc_dot*v = 0 => Jc*Minv*(S'*tau + Jc'*fc - h) + Jc_dot*v = 0
            d vc_dot/dq -> getFrameAccelerationDerivatives
            d vc_dot/dv -> getFrameAccelerationDerivatives
            d vc_dot/dtau = Jc*Minv*S'
            d vc_dot/dfc = Jc*Minv*Jc'
            vc_dot ?= vc_dot(xc, uc) + d vc_dot/dx (x-xc) + d vc_dot/du (u-uc) = 0

            [d vc_dot/dx    0   0   d vc_dot/du] [x1]
                                                 [x2]   = [-vc_dot(xc,uc) + d vc_dot/dx *xc + d vc_dot/du *uc]
                                                 [u0]
                                                 [u1]
        */

        //pinocchio::computeForwardKinematicDerivatives();
        Eigen::MatrixXd da_dq = Eigen::MatrixXd::Zero(3*4, v_dim);
        Eigen::MatrixXd da_dv = Eigen::MatrixXd::Zero(3*4, v_dim);

        std::cout <<"q_propagated[i]:\n" <<q_propagated[i] <<std::endl;
        std::cout <<"v_propagated[i]:\n" <<v_propagated[i] <<std::endl;

        Eigen::VectorXd v_dot_eq = Eigen::VectorXd::Zero(v_dim);
        v_dot_eq = robot.get_robot_data().Minv*(robot.get_S().transpose()*robot.get_optimal_torques()[i] + Jc_in_time[i].transpose()*robot.get_optimal_contact_forces()[i] - robot.get_robot_data().nle);
        std::cout <<"v_dot_eq:\n" <<v_dot_eq <<std::endl;

        robot.compute_terms(q_propagated[i], v_propagated[i]);
        robot.compute_second_order_FK(q_propagated[i], v_propagated[i], v_dot_eq);
        robot.get_Jc_dot_times_v(Jc_dot_times_v, contact_feet_names, contact_feet_index);

        pinocchio::computeForwardKinematicsDerivatives(robot.get_robot_model(), robot.get_robot_data(), q_propagated[i], v_propagated[i], v_dot_eq);
        robot.get_contact_acceleration_derivatives(da_dq, da_dv, contact_feet_names, contact_feet_index);
        std::cout <<"da_dq\n" <<da_dq <<std::endl;

        auto qw = q_propagated[i](6);
        auto qx = q_propagated[i](3);
        auto qy = q_propagated[i](4);
        auto qz = q_propagated[i](5);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;        // Dynamics of quaternions: quat_dot = 0.5*Q*ang_vel
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(q_dim, v_dim);
        G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3,3);                                           //         [p_b_dot]   [  v_b  ]   [I 0 0] [  v_b  ] 
        G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);      // q_dot = [q_b_dot] = [ Q*w_b ] = [0 Q 0] [  w_b  ]
        G.block(3, 3, 4, 3) = 0.5*Q;                                                                      //         [q_j_dot]   [q_j_dot]   [0 0 I] [q_j_dot]

        std::cout <<"G\n" <<G <<std::endl;
        std::cout <<"da_dq * Ginv\n" <<da_dq*G.completeOrthogonalDecomposition().pseudoInverse() <<std::endl;

        std::cout <<"da_dv:\n" <<da_dv <<std::endl;

        Eigen::MatrixXd df_du = Eigen::MatrixXd::Zero(12, joint_dim+contact_forces_dim);
        df_du.leftCols(joint_dim) = Jc_in_time[i]*robot.get_robot_data().Minv*robot.get_S().transpose();
        df_du.rightCols(contact_forces_dim) = Jc_in_time[i]*robot.get_robot_data().Minv*(Jc_in_time[i].transpose());

        Eigen::VectorXd uc = Eigen::VectorXd::Zero(joint_dim+contact_forces_dim);
        uc.head(joint_dim) = robot.get_optimal_torques()[i];
        uc.tail(contact_forces_dim) = robot.get_optimal_contact_forces()[i];

        /*
                [                                   ] [x1]
                [ |da_dq*Ginv  da_dv|  0   0   df_du] [x2] = [da_dq*Ginv*qc + da_dv*vc + df_du*uc - J_dot_times_v]
                                                      [u0]
                                                      [u1]
        */
        
        A.block(i*12, (i-1)*(q_dim+v_dim), 12, q_dim) = da_dq*G.completeOrthogonalDecomposition().pseudoInverse();
        A.block(i*12, (i-1)*(q_dim+v_dim)+q_dim, 12, v_dim) = da_dv;
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_forces_dim), 12, joint_dim+contact_forces_dim) = df_du;

        b.segment(i*12, 12) = da_dq*G.completeOrthogonalDecomposition().pseudoInverse()*q_propagated[i] + da_dv*v_propagated[i] + df_du*uc - Jc_dot_times_v;

        // #################################################################################################################

    }

    Task contact_constraint(A, b, D, f);
    //std::cout <<"Contact constraint:\n" <<contact_constraint <<std::endl;
    return contact_constraint;
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

            [  q  ]   [x1]             [x1_dot]   [q_dot]   [                G(x1)*x2                  ]   [f1(x, u)]
        x = [     ] = [  ]  -> x_dot = [      ] = [     ] = [                                          ] = [        ] = f(x, u)
            [  v  ]   [x2]             [x2_dot]   [v_dot]   [   Minv(x1,x2)*(S'*tau+Jc'*fc-h(x1,x2))   ]   [f2(x, u)]

        x_(k+1) = x_k + dT*f(x_k, u_k) -> [x1_(k+1)] = [                 x1_k + dT*G*x2_k                         ]
                                          [x2_(k+1)]   [x2_k + dT*Minv(x1_k,x2_k)*(S'*tau_k+Jc'*fc_k-h(x1_k,x2_k))]
        Linearizing around xc, uc

        x_dot = f(xc, uc) + d f(x,u)/dx |(xc, uc) *(x-xc) + d f(x,u)/du |(xc, uc) *(u-uc)
        x-xc = x_hat,   u-uc=u_hat
        d f1/dx = [d f1/dx1, d f2/dx2]
        d f2/dx = [d f2/dx1, d f2/dx2]

        d f1/dx1 = d G/dx1,       d f1/dx2 = (G+dG_dv*v)*dT
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
Task MPC::dynamic_constraint(GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){
    
    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = robot.get_contact_feet_dim()*3;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*(q_dim+v_dim), mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*(q_dim+v_dim));
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_forces_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(v_dim);
    tau.tail(joint_dim) = this->robot.get_optimal_torques()[0];
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
    G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3,3);                                           //         [p_b_dot]   [  v_b  ]   [I 0 0] [  v_b  ] 
    G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);      // q_dot = [q_b_dot] = [ Q*w_b ] = [0 Q 0] [  w_b  ]
    G.block(3, 3, 4, 3) = 0.5*Q;                                                                      //         [q_j_dot]   [q_j_dot]   [0 0 I] [q_j_dot]

    Eigen::MatrixXd dQ_dq = Eigen::MatrixXd::Zero(4, 4);
    dQ_dq.row(0) << 0.5*v(3), 0, 0.5*v(5), -0.5*v(4);   // q1_dot = 0.5*(wx*q0-wy*q3+wz*q2) => d q1_dot/dq = 0.5*[wx 0 wz -wy]
    dQ_dq.row(1) << 0.5*v(4), -0.5*v(5), 0, 0.5*v(3);   // q2_dot = 0.5*(wy*q0-wz*q1+wx*q3) => d q2_dot/dq = 0.5*[wy -wz 0 wx]        
    dQ_dq.row(2) << 0.5*v(5), 0.5*v(4), -0.5*v(3), 0;   // q3_dot = 0.5*(wz*q0+wy*q1-wx*q2) => d q3_dot/dq = 0.5*[wz wy -wx 0]
    dQ_dq.row(3) << 0, -0.5*v(3), -0.5*v(4), -0.5*v(5); // q0_dot = -0.5*(wx*q1+wy*q2+wz*q3) => d q0_dot/dq = -0.5*[0 wx wy wz]

    Eigen::MatrixXd dG_dq = Eigen::MatrixXd::Zero(q_dim, q_dim);
    dG_dq.block(3, 3, 4, 4) = dQ_dq;

    //robot.compute_terms(q, v);

    // A0 = [   0      G  ]     =>  A0*x0 = [        G*v0       ]
    //      [ddq_dq ddq_dv]                 [ddq_dq*q0+ddq_dv*v0]
    A0.topLeftCorner(q_dim, q_dim) = dG_dq;
    A0.topRightCorner(q_dim, v_dim) = G;
    pinocchio::computeABADerivatives(robot.get_robot_model(), robot.get_robot_data(), q, v, tau);
    Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd ddq_dq = robot.get_robot_data().ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
    Eigen::MatrixXd ddq_dv = robot.get_robot_data().ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

    //pinocchio::nonLinearEffects(robot.get_robot_model(), robot.get_robot_data(), q, v);
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(joint_dim, v_dim);
    S.rightCols(joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);
    //std::cout <<"Selection matrix of the actuated d.o.f:\n" <<S <<std::endl;

    A0.bottomLeftCorner(v_dim, q_dim) = ddq_dq;
    A0.bottomRightCorner(v_dim, v_dim) = ddq_dv;

    x0.head(q_dim) = q;     // x0 = first linearization point = [q0]
    x0.tail(v_dim) = v;     //                                  [v0]

    this->Minv = robot.get_robot_data().Minv;

    // f(x) = non-linear dynamics = [            G*v          ] => f(x0) = f(x) computed for x=x0
    //                              [Minv*(S'*tau+Jc'*fc-h(x))]
    f_x0.head(q_dim) = G*v;
    //f_x0.tail(v_dim) = -robot.get_robot_data().Minv*robot.get_robot_data().nle;
    f_x0.tail(v_dim) = robot.get_robot_data().Minv*(tau + Jc_in_time[0].transpose()*fc) - robot.get_robot_data().Minv*robot.get_robot_data().nle;    // M*v_dot + h = tau + Jc*fc

    /*
        [I 0      0          0    ] [q1]   [q0]   [ G*v0  ]*dT    [q1] = q0 + G*v0*dT
        [0 I -dT*Minv -dT*Minv*Jc'] [v1] = [v0] + [-Minv*h]    => [v1] = v0 + dT*Minv*(t0+Jc'*f0-h)
                                    [t1]
                                    [f1]
    */

    B.bottomLeftCorner(v_dim, joint_dim) = dT*robot.get_robot_data().Minv.rightCols(joint_dim);
    B.bottomRightCorner(v_dim, contact_forces_dim) = dT*robot.get_robot_data().Minv*Jc_in_time[0].transpose();

    A.topLeftCorner(q_dim, q_dim) = Eigen::MatrixXd::Identity(q_dim, q_dim);
    A.block(q_dim, q_dim, v_dim, v_dim) = Eigen::MatrixXd::Identity(v_dim, v_dim);
    A.block(0, mpc_step_horizon*(q_dim+v_dim), q_dim+v_dim, joint_dim+contact_forces_dim) = -B;

    Eigen::VectorXd u_c = Eigen::VectorXd::Zero(joint_dim+contact_forces_dim);
    u_c << tau.tail(joint_dim), fc;

    // B*u_c = [  0         0   ] * [tau] = dT * [        0 + 0       ] = dT * [         0          ]
    //         [Minv    Minv*Jc']   [fc ]        [Minv*tau+Minv*Jc'*fc]        [Minv*tau+Minv*Jc'*fc]

    //std::cout <<"nle:\n" <<data.nle <<std::endl;

    //b.head(q_dim+v_dim) = x0 + f_x0*dT;
    b.head(q_dim+v_dim) = x0 + f_x0*dT - B*u_c;
    // => [I -B][x_(k+1)] = [x0] + dT*[         G*v         ] - B*uc   -> x_(k+1) = x0 + B*(u_k - u_c) + dT*[         G*v         ]
    //          [ u_(k) ]             [M^(-1)*(tau+Jc'*fc-h)]                                               [M^(-1)*(tau+Jc'*fc-h)]

    Eigen::VectorXd f_xi = Eigen::VectorXd::Zero(q_dim+v_dim);
    for(int i=1; i<mpc_step_horizon; i++){
        B.setZero();
        Q.setZero();
        Ai.setZero();
        f_xi.setZero();
        u_c.setZero();
        q = q_propagated[i];
        v = v_propagated[i];
        tau.tail(joint_dim) = this->robot.get_optimal_torques()[i]; 

        x0.head(q_dim) = q;
        x0.tail(v_dim) = v;

        qw = q(6);
        qx = q(3);
        qy = q(4);
        qz = q(5);
        Q.row(0) << qw, -qz, qy;            // q0_dot = -0.5*(wx*q1+wy*q2+wz*q3)       [q1_dot]         [ q0 -q3  q2] [wx]
        Q.row(1) << qz, qw, -qx;            // q1_dot = 0.5*(wx*q0-wy*q3+wz*q2)    =>  [q2_dot] =  0.5* [ q3  q0 -q1] [wy]
        Q.row(2) << -qy, qx, qw;            // q2_dot = 0.5*(wy*q0-wz*q1+wx*q3)        [q3_dot]         [-q2  q1  q0] [wz]
        Q.row(3) << -qx, -qy, -qz;          // q3_dot = 0.5*(wz*q0+wy*q1-wx*q2)        [q0_dot]         [-q1 -q2 -q3]

        dQ_dq.row(0) << 0.5*v(3), 0, 0.5*v(5), -0.5*v(4);
        dQ_dq.row(1) << 0.5*v(4), -0.5*v(5), 0, 0.5*v(3);
        dQ_dq.row(2) << 0.5*v(5), 0.5*v(4), -0.5*v(3), 0;
        dQ_dq.row(3) << 0, -0.5*v(3), -0.5*v(4), -0.5*v(5);

        dG_dq.block(3, 3, 4, 4) = dQ_dq;
        
        robot.compute_terms(q, v);
        
        pinocchio::computeABADerivatives(robot.get_robot_model(), robot.get_robot_data(), q, v, tau);
        Eigen::MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd ddq_dq = robot.get_robot_data().ddq_dq*Ginv;    // Partial derivative of the joint acceleration vector with respect to the joint configuration.
        Eigen::MatrixXd ddq_dv = robot.get_robot_data().ddq_dv;    // Partial derivative of the joint acceleration vector with respect to the joint velocity.

        Ai.topLeftCorner(q_dim, q_dim) = dT*dG_dq;
        Ai.topRightCorner(q_dim, v_dim) = dT*G;
        Ai.bottomLeftCorner(v_dim, q_dim) = dT*ddq_dq;          // df/dx = A = [dG_dq     G   ]
        Ai.bottomRightCorner(v_dim, v_dim) = dT*ddq_dv;         //             [ddq_dq  ddq_dv]

        B.bottomLeftCorner(v_dim, joint_dim) = dT*robot.get_robot_data().Minv*S.transpose();                           // df/du = B = [   0        0     ]
        B.bottomRightCorner(v_dim, contact_forces_dim) = dT*robot.get_robot_data().Minv*Jc_in_time[i].transpose();     //             [Minv*S'   Minv*Jc']

        f_xi.head(q_dim) = G*v;
        f_xi.tail(v_dim) = robot.get_robot_data().Minv*(tau + Jc_in_time[i].transpose()*fc) - robot.get_robot_data().Minv*robot.get_robot_data().nle;    // M*v_dot + h = tau + Jc*fc

        Eigen::VectorXd u_c = Eigen::VectorXd::Zero(joint_dim+contact_forces_dim);
        u_c << tau.tail(joint_dim), fc;

        /*
            x_k+1 = x_k + {f(x_c, u_c) + A*(x_k - x_c) + B*(u_k - u_c)}*dT

            [ I       0    -B0    0 ] [x1]   [      x1 - B0*u0       ]   [x0 + f(xc,uc) + A0*(x0-xc) - B0*uc)]    [x1]   [x0 + dT*{f(xc, uc)+A0*(x0-xc)+B0*(u0-uc)}]
            [-A1-I    I     0    -B1] [x2] = [x2 - x1 - A1*x1 - B1*u1] = [    f(xc, uc) - A1*xc - B1*uc      ] => [x2] = [x1 + dT*{f(xc, uc)+A1*(x1-xc)+B1*(u1-uc)}]
                                      [u0]
                                      [u1]
        */

        A.block(i*(q_dim+v_dim), i*(q_dim+v_dim), q_dim+v_dim, q_dim+v_dim) = Eigen::MatrixXd::Identity(q_dim+v_dim, q_dim+v_dim);      //x_k+1
        A.block(i*(q_dim+v_dim), (i-1)*(q_dim+v_dim), q_dim+v_dim, q_dim+v_dim) = -Ai - Eigen::MatrixXd::Identity(q_dim+v_dim, q_dim+v_dim);    // x_k + A*x_k
        //A.block(i*(q_dim+v_dim), (i-1)*(q_dim+v_dim), q_dim+v_dim, q_dim+v_dim) = - Eigen::MatrixXd::Identity(q_dim+v_dim, q_dim+v_dim);
        A.block(i*(q_dim+v_dim), mpc_step_horizon*(q_dim+v_dim)+i*(joint_dim+contact_forces_dim), q_dim+v_dim, joint_dim+contact_forces_dim) = -B;  //B*u_k

        b.segment(i*(q_dim+v_dim), q_dim+v_dim) = f_xi*dT - B*u_c - Ai*x0;
        //std::cout <<"b:\n" <<b <<std::endl;
    }

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
        [0 ... 0 I^A 0 ... 0] [     [q_1', qdot_1']'     ]    [u.b]
        [0 ... 0 0 I^A ... 0] [         [...]            ]    [u.b]
        [         ...       [ [ [q_(N-1)', qdot_(N-1)']' ] <= [u.b]         I^A = [I, 0]
        [0 ... 0 0 0 ... I^A] [     [tau_1', fc_1']'     ]    [u.b]               [0, 0]
                              [         [...]            ]    
                              [ [tau_(N-1)', fc_(N-1)']  ]    
    */

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;
    int contact_forces_dim = 3*robot.get_contact_feet_dim();  // 3 joints for each leg contacting the terrain
    int cols = mpc_step_horizon * (q_dim + v_dim + joint_dim + contact_forces_dim);
    
    Eigen::MatrixXd A = MatrixXd::Zero(0, cols);
    Eigen::VectorXd b = VectorXd::Zero(0);
    Eigen::MatrixXd D = MatrixXd::Zero(2*mpc_step_horizon*joint_dim, cols);
    Eigen::VectorXd f = VectorXd::Zero(2*mpc_step_horizon*joint_dim);

    for(int i = 0; i<mpc_step_horizon; i++){
        D.block(i*2*joint_dim, (q_dim+v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim, joint_dim) = MatrixXd::Identity(joint_dim, joint_dim);
        D.block(i*2*joint_dim+joint_dim, (q_dim+v_dim)*mpc_step_horizon + i*(joint_dim+contact_forces_dim), joint_dim, joint_dim) = -MatrixXd::Identity(joint_dim, joint_dim);
        f.segment(i*2*joint_dim, joint_dim) = VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_max();
        f.segment(i*2*joint_dim + joint_dim, joint_dim) = -VectorXd::Ones(mpc_step_horizon*(joint_dim))*robot.get_tau_min();
    
    }

    Task torque_limits(A, b, D, f);
    //std::cout <<"Torque constraint:\n" <<torque_limits <<std::endl;
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
    /*
   //std::cout <<"Stampa motion tracking\n" <<std::endl;
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
        //std::cout <<"i motion track: " <<i <<std::endl;
        //joint_des_pos = robot.clik_alg(gen_poses.generalized_poses_with_time[i].gen_pose, robot.get_robot_model(), robot.get_robot_data());
        // std::cout <<"joint_des_pos:\n" <<joint_des_pos <<"\n" <<std::endl;
        //joint_des_vel = (joint_des_pos-joint_des_pos_prev)/dT;
        // std::cout <<"joint_des_vel:\n" <<joint_des_vel <<"\n" <<std::endl;
        joint_des_pos_prev = joint_des_pos;
        //std::cout <<"Posizioni desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos <<"\n";
        //std::cout <<"Velocità desiderate dei giunti:\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel <<"\n";

        /*
        b.segment(2*i*(joint_dim), joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = gen_poses.generalized_poses_with_time[i].gen_pose.joint_vel;
        

        b.segment(2*i*(joint_dim), joint_dim) = joint_des_pos;
        b.segment(2*i*(joint_dim)+joint_dim, joint_dim) = joint_des_vel;

    }
    // std::cout <<"b: \n" <<b <<"\n" <<std::endl;

     Task motion_tracking(A, b, D, f);
     return motion_tracking;
     */
    
}

// ################################################################################################################################
// #                                                 MOTION TRACKING OF BASE                                                      #
// ################################################################################################################################

/*
        [vb_dot]                                                 [vb_dot_ref]        [vb_ref - vb]        [  pb_ref - pb   ]
        [      ] = Jb*v_dot + Jb_dot*v = Jb*Minv(tau+Jc'*fc-h) = [          ] + Kd * [           ] + Kp * [                ] - Jb_dot*v
        [wb_dot]                                                 [wb_dot_ref]        [wb_ref - wb]        [Q^+ *(qb_ref-qb)]

        [0 0 Jb*Minv Jb*Minv*Jc'] [ q1 ]   
                                  [ v1 ] = Jb*Minv*h + v_dot_ref + Kp*(p_ref - p) + Kd*(v_ref - v) - Jb_dot*v
                                  [tau0] 
                                  [fc0 ]
                    
            => Jb*Minv*(tau_0 + Jc'*fc_0 - h) + Jb_dot*v = (...)
            => Jb*v_dot + Jb_dot*v = (...)
*/

Task MPC::motion_tracking_base_constraint(GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){
    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim-6;

    Eigen::VectorXd q = q_;
    Eigen::VectorXd v = v_;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6*mpc_step_horizon, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6*mpc_step_horizon);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, (q_dim+v_dim+joint_dim+contact_dim)*mpc_step_horizon);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    robot.compute_terms(q, v);
    robot.compute_second_order_FK(q, v);

    Eigen::MatrixXd Jb = Eigen::MatrixXd::Zero(6, v_dim);
    Eigen::MatrixXd Jb_dot = Eigen::MatrixXd::Zero(6, v_dim);
    Eigen::VectorXd Jb_dot_times_v = Eigen::VectorXd::Zero(v_dim);
    robot.get_Jb(Jb);
    robot.get_Jb_dot(Jb_dot);
    robot.get_Jb_dot_times_v(Jb_dot_times_v);

    A.block(0, mpc_step_horizon*(q_dim+v_dim), 6, joint_dim) = Jb*robot.get_robot_data().Minv.rightCols(joint_dim);
    A.block(0, mpc_step_horizon*(q_dim+v_dim)+joint_dim, 6, contact_dim) = Jb*robot.get_robot_data().Minv*Jc_in_time[0].transpose();
    
    pinocchio::FrameIndex base_id = 1;

    Eigen::VectorXd base_velocity = Eigen::VectorXd::Zero(6);
    base_velocity = Jb*v;

    b.segment(0, 3) = gen_poses.generalized_poses_with_time[0].gen_pose.base_acc 
                    + Kd_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[0].gen_pose.base_vel - v.head(3) )
                    + Kp_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[0].gen_pose.base_pos - q.head(3) )
                    - Jb_dot_times_v.head(3);

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

    b.segment(3, 3) = Kd_ang.asDiagonal()*(gen_poses.generalized_poses_with_time[0].gen_pose.base_angvel - v.segment(3, 3))
                    + Kp_ang.asDiagonal()*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) )
                    - Jb_dot_times_v.tail(3);
    b.head(6) = b.head(6).eval() + Jb*robot.get_robot_data().Minv*robot.get_robot_data().nle;

    for (int i=1; i<mpc_step_horizon; i++){
        Jb_dot_times_v.setZero();
        Jb.setZero();

        q = q_propagated[i];
        v = v_propagated[i];

        auto qw = q(6);
        auto qx = q(3);
        auto qy = q(4);
        auto qz = q(5);

        Quaterniond quat = Quaternion<double>(qw, qx, qy, qz);
        Quaterniond quat_des = Quaternion<double>(gen_poses.generalized_poses_with_time[i].gen_pose.base_quat.w(), gen_poses.generalized_poses_with_time[i].gen_pose.base_quat.x(), gen_poses.generalized_poses_with_time[i].gen_pose.base_quat.y(), gen_poses.generalized_poses_with_time[i].gen_pose.base_quat.z());

        robot.compute_terms(q, v);
        robot.compute_second_order_FK(q, v);
        robot.get_Jb(Jb);
        robot.get_Jb_dot_times_v(Jb_dot_times_v);

        /*
        A.block(6*i, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), 6, joint_dim) = Jb*robot.get_robot_data().Minv.rightCols(joint_dim);
        A.block(6*i, mpc_step_horizon*(q_dim+v_dim)+joint_dim + i*(joint_dim+contact_dim), 6, contact_dim) = Jb*robot.get_robot_data().Minv*Jc_in_time[i].transpose();

        b.segment(i*6, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_acc 
                            + Kd_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_vel - v.head(3) )
                            + Kp_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_pos - q.head(3))
                            - Jb_dot_times_v.head(3);

        b.segment(3+i*6, 3) = Kd_ang.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel - v.segment(3, 3))
                            + Kp_ang.asDiagonal()*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) )
                            - Jb_dot_times_v.tail(3);

        b.segment(i*6, 6) = b.segment(i*6, 6).eval() + Jb*robot.get_robot_data().Minv*robot.get_robot_data().nle;
        */

        // ##########################################################################################################################################

        /*
        qw = q(6);
        qx = q(3);
        qy = q(4);
        qz = q(5);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;        // Dynamics of quaternions: quat_dot = 0.5*Q*ang_vel
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;
        /*
            f(x,u) = J_b*[Minv*tau + Minv*Jc'*fc - Minv*h] + Jb_dot*v - v_dot_des - Kp*(p_des-p) - Kd*(v_des-v)

            df_dx = [Kp     0      0    Kd     0        0],     df_du = [Jb*Minv    Jb*Minv*Jc']
                    [0   Kp_ang    0     0     Kd_ang   0]

            f(x,u) ?= f(xc, uc) + df_dx|(xc,uc) *(x-xc) + df_du|(xc,uc) * (u-uc)
        
        robot.get_Jb_dot(Jb_dot);
        Eigen::MatrixXd df_dx = Eigen::MatrixXd::Zero(6, q_dim+v_dim);
        df_dx.block(0, 0, 3, 3) = Kp_pos.asDiagonal();
        df_dx.block(0, q_dim, 3, 3) = Kd_pos.asDiagonal();
        df_dx.topRightCorner(3, v_dim) = df_dx.topRightCorner(3, v_dim).eval() + Jb_dot;
        df_dx.block(3, 3, 3, 4) = Kp_ang.asDiagonal()*Q.completeOrthogonalDecomposition().pseudoInverse();
        df_dx.block(3, q_dim+3, 3, 3) = Kd_ang.asDiagonal();

        Eigen::MatrixXd df_du = Eigen::MatrixXd::Zero(6, joint_dim+contact_dim);
        df_du.block(0, 0, 6, joint_dim) = Jb*robot.get_robot_data().Minv.rightCols(joint_dim);
        df_du.block(0, joint_dim, 6, contact_dim) = Jb*robot.get_robot_data().Minv*Jc_in_time[i].transpose();

        Eigen::VectorXd ref = Eigen::VectorXd::Zero(6);
        ref.head(3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_acc 
                            + Kd_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_vel - v.head(3) )
                            + Kp_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_pos - q.head(3));
        ref.tail(3) = Kd_ang.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel - v.segment(3, 3))
                            + Kp_ang.asDiagonal()*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) );

        Eigen::VectorXd a1 = -ref + Jb*robot.get_robot_data().Minv.rightCols(joint_dim)*robot.get_optimal_torques()[i];
        //std::cout <<"a1:\n" <<a1 <<std::endl;
        Eigen::VectorXd a2 = a1 + Jb*robot.get_robot_data().Minv*Jc_in_time[i].transpose()*robot.get_optimal_contact_forces()[i];
        //std::cout <<"a1:\n" <<a2 <<std::endl;
        Eigen::VectorXd a3 = a2 - Jb*robot.get_robot_data().Minv*robot.get_robot_data().nle;
        //std::cout <<"a3:\n" <<a3 <<std::endl;
        a3 = a3.eval() + Jb_dot*v;

        Eigen::VectorXd xc = Eigen::VectorXd::Zero(q_dim+v_dim);
        Eigen::VectorXd uc = Eigen::VectorXd::Zero(joint_dim+contact_dim);
        xc.head(q_dim) = q;
        xc.tail(v_dim) = v;
        uc.head(joint_dim) = robot.get_optimal_torques()[i].tail(joint_dim);
        uc.tail(contact_dim) = robot.get_optimal_contact_forces()[i];

        A.block(6*i, i*(q_dim+v_dim), 6, q_dim+v_dim) = df_dx;
        A.block(6*i, mpc_step_horizon*(q_dim+v_dim)+i*(joint_dim+contact_dim), 6, joint_dim+contact_dim) = df_du;
        b.segment(i*6, 6) = -a3 + df_dx*xc + df_du*uc;
        //std::cout <<"b:\n" <<b <<std::endl;
        */
        
        //###########################################################################################################################################

        //###########################################################################################################################################

        
        Eigen::MatrixXd da_dq = Eigen::MatrixXd::Zero(6, v_dim);
        Eigen::MatrixXd da_dv = Eigen::MatrixXd::Zero(6, v_dim);

        Eigen::VectorXd v_dot_eq = Eigen::VectorXd::Zero(v_dim);
        v_dot_eq = robot.get_robot_data().Minv*(robot.get_S().transpose()*robot.get_optimal_torques()[i] + Jc_in_time[i].transpose()*robot.get_optimal_contact_forces()[i] - robot.get_robot_data().nle);
        
        robot.compute_terms(q, v);
        robot.compute_second_order_FK(q, v, v_dot_eq);
        robot.get_Jb(Jb);
        robot.get_Jb_dot(Jb_dot);
        robot.get_Jb_dot_times_v(Jb_dot_times_v);
        
        pinocchio::computeForwardKinematicsDerivatives(robot.get_robot_model(), robot.get_robot_data(), q_propagated[i], v_propagated[i], v_dot_eq);
        
        robot.get_base_acceleration_derivatives(da_dq, da_dv);

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(q_dim, v_dim);
        G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3,3);                                           //         [p_b_dot]   [  v_b  ]   [I 0 0] [  v_b  ] 
        G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);      // q_dot = [q_b_dot] = [ Q*w_b ] = [0 Q 0] [  w_b  ]
        G.block(3, 3, 4, 3) = 0.5*Q;                                                                      //         [q_j_dot]   [q_j_dot]   [0 0 I] [q_j_dot]

        //da_dq = da_dq.eval()*G.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::MatrixXd df_du = Eigen::MatrixXd::Zero(6, joint_dim+contact_dim);
        df_du.leftCols(joint_dim) = Jb*robot.get_robot_data().Minv*robot.get_S().transpose();
        df_du.rightCols(contact_dim) = Jb*robot.get_robot_data().Minv*(Jc_in_time[i].transpose());

        Eigen::VectorXd uc = Eigen::VectorXd::Zero(joint_dim+contact_dim);
        uc.head(joint_dim) = robot.get_optimal_torques()[i];
        uc.tail(contact_dim) = robot.get_optimal_contact_forces()[i];

        Eigen::MatrixXd K1 = Eigen::MatrixXd::Zero(6, 6);
        K1.topLeftCorner(3, 3) = Kp_pos.asDiagonal();
        K1.bottomRightCorner(3, 3) = Kp_ang.asDiagonal();
        Eigen::MatrixXd K2 = Eigen::MatrixXd::Zero(6, 6);
        K2.topLeftCorner(3, 3) = Kd_pos.asDiagonal();
        K2.bottomRightCorner(3, 3) = Kd_ang.asDiagonal();

    /*
        Eigen::VectorXd f_xc = Eigen::VectorXd::Zero(6);
        f_xc.head(3) = Jb_dot_times_v.head(3) - gen_poses.generalized_poses_with_time[i].gen_pose.base_acc.head(3) 
                    - Kd_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_vel - v_propagated[i].head(3) )
                    - Kp_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_pos - q_propagated[i].head(3) );

        f_xc.tail(3) = Jb_dot_times_v.tail(3)
                    - Kd_ang.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel - v_propagated[i].segment(3, 3))
                    - Kp_ang.asDiagonal()*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) );
        
        A.block(i*6, (i-1)*(q_dim+v_dim), 6, q_dim) = da_dq*G.completeOrthogonalDecomposition().pseudoInverse() + K1*Jb + K2*Jb_dot;    // df_dq = da_dq - Kp*Jb
        A.block(i*6, (i-1)*(q_dim+v_dim)+q_dim, 6, v_dim) = da_dv + K2*Jb;
        */

        Eigen::VectorXd f_xc = Eigen::VectorXd::Zero(6);
        f_xc.head(3) = gen_poses.generalized_poses_with_time[i].gen_pose.base_acc.head(3) 
                    + Kd_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_vel - v_propagated[i].head(3) )
                    + Kp_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_pos - q_propagated[i].head(3) );

        f_xc.tail(3) = Kd_ang.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.base_angvel - v_propagated[i].segment(3, 3))
                    + Kp_ang.asDiagonal()*( pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()) );
        
        A.block(i*6, (i-1)*(q_dim+v_dim), 6, q_dim) = (K1*Jb + K2*Jb_dot)*G.completeOrthogonalDecomposition().pseudoInverse();
        A.block(i*6, (i-1)*(q_dim+v_dim)+q_dim, 6, v_dim) = K2*Jb;

        A.block(i*6, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), 6, joint_dim+contact_dim) = df_du;
        b.segment(i*6, 6) = (K1*Jb+K2*Jb_dot)*G.completeOrthogonalDecomposition().pseudoInverse()*q_propagated[i] + (K2*Jb)*v_propagated[i] - df_du*uc + f_xc -Jb_dot_times_v;
        

        //###########################################################################################################################################
    }

    Task motion_tracking(A, b, D, f);
    std::cout <<"Task motion tracking base:\n" <<motion_tracking <<"\n" <<std::endl;
    return motion_tracking;
}

// ################################################################################################################################
// #                                             MOTION TRACKING OF SWING FEET                                                    #
// ################################################################################################################################

Task MPC::motion_tracking_swing_feet_constraint(GeneralizedPosesWithTime gen_poses, Eigen::VectorXd q_, Eigen::VectorXd v_){

    int q_dim = robot.get_state_dim() +1;
    int v_dim = robot.get_state_dim();
    int contact_dim = 3*robot.get_contact_feet_dim();
    int joint_dim = v_dim - 6;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(mpc_step_horizon*(3*4), mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(mpc_step_horizon*12);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(0, mpc_step_horizon*(q_dim+v_dim+joint_dim+contact_dim));
    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);

    Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(3*4, robot.get_robot_model().nv);
    Eigen::MatrixXd Js_dot = Eigen::MatrixXd::Zero(3*4, robot.get_robot_model().nv);
    Eigen::VectorXd Js_dot_times_v = Eigen::VectorXd::Zero(12);

    Eigen::VectorXd feet_vel = Eigen::VectorXd::Zero(12);

    if (gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names.size() == 4){
        A.resize(0, NoChange);
        b.resize(0, NoChange);
        Task motion_tracking_swing_feet(A, b, D, f);
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

        swing_feet_names.resize(4-contact_feet_names.size());
        swing_feet_index.resize(4-contact_feet_names.size());

        int counter = 0;
        for (size_t j= 0; j < 4; j++) {
            if ( std::find(contact_feet_names.begin(), contact_feet_names.end(), robot_short_feet_names[j]) == contact_feet_names.end() ) {
                // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing foot.
                swing_feet_index[counter] = j;
                swing_feet_names[counter] = robot_feet_names[j];
                counter++;
            }
        }

        robot.compute_terms(q_propagated[i], v_propagated[i]);
        robot.compute_second_order_FK(q_propagated[i], v_propagated[i]);
        //robot.compute_terms(robot.get_optimal_q()[i], robot.get_optimal_v()[i]);
        //robot.compute_second_order_FK(robot.get_optimal_q()[i], robot.get_optimal_v()[i]);

        robot.get_Js(Js, swing_feet_names, swing_feet_index);
        robot.get_Js_dot(Js_dot, swing_feet_names, swing_feet_index);
        robot.get_Js_dot_times_v(Js_dot_times_v, swing_feet_names, swing_feet_index);
        //std::cout <<"Js_dot_times_v:\n" <<Js_dot_times_v <<std::endl;
        
        feet_vel = Js*v_propagated[i];

        /*
        for(int k=0; k<swing_feet_index.size(); k++){
            std::cout <<k <<std::endl;
            pinocchio::FrameIndex foot_id = robot.get_robot_model().getFrameId(swing_feet_names[k]);
            Eigen::VectorXd swing_foot_vel = pinocchio::getFrameVelocity(robot.get_robot_model(), robot.get_robot_data(), foot_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
            Eigen::VectorXd foot_pos = robot.get_robot_data().oMf[foot_id].translation();

            b.segment(i*12 + swing_feet_index[k]*3, 3) = gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.segment(k*3, 3)
                + Kd_s_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(k*3, 3) - feet_vel.segment(3*swing_feet_index[k], 3))
                + Kp_s_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(k*3, 3) - foot_pos.topRows(3))
                - Js_dot_times_v.segment(3*swing_feet_index[k], 3);

            
            //std::cout <<"Foot name: " <<swing_feet_names[k] <<std::endl;
            //std::cout <<"Desired foot acc\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.segment(k*3, 3) <<std::endl;
            //std::cout <<"Desired foot vel\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(k*3, 3) <<std::endl;
            //std::cout <<"Desired foot pos\n" <<gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(k*3, 3) <<std::endl;
            //std::cout <<"Actual foot vel\n" <<feet_vel.segment(3*swing_feet_index[k], 3) <<std::endl;
            //std::cout <<"Actual foot pos\n" <<foot_pos.topRows(3) <<std::endl;
            

            Eigen::VectorXd e_a = gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.segment(k*3, 3) - Js_dot_times_v.segment(3*swing_feet_index[k], 3);
            Eigen::VectorXd e_v = gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(k*3, 3) - swing_foot_vel.topRows(3);
            Eigen::VectorXd e_p = gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(k*3, 3) - foot_pos.topRows(3);
        }

        /*
            [0 0 Js*Minv Js*Minv*Jc'][q_k+1] = Js*Minv*h + ref -Js_dot*v => Js*(Minv*tau_k + Minv*Jc'*fc_k - Minv*h) = ref - Js_dot*v => Js*v_dot = ref - Js_dot*v
                                     [v_k+1]
                                     [tau_k]
                                     [fc_k ]
        
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), 12, joint_dim) = Js*robot.get_robot_data().Minv.rightCols(joint_dim);
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim)+joint_dim+i*(joint_dim+contact_dim), 12, contact_dim) = Js*robot.get_robot_data().Minv*(Jc_in_time[i].transpose());

        b.segment(i*12, 12) = b.segment(i*12, 12).eval() + Js*robot.get_robot_data().Minv*robot.get_robot_data().nle - Js_dot_times_v;
        */

       // ###########################################################################################################

        Eigen::MatrixXd da_dq = Eigen::MatrixXd::Zero(12, v_dim);
        Eigen::MatrixXd da_dv = Eigen::MatrixXd::Zero(12, v_dim);
        Eigen::VectorXd v_dot_eq = Eigen::VectorXd::Zero(v_dim);
        v_dot_eq = robot.get_robot_data().Minv*(robot.get_S().transpose()*robot.get_optimal_torques()[i] + Jc_in_time[i].transpose()*robot.get_optimal_contact_forces()[i] - robot.get_robot_data().nle);
        pinocchio::computeForwardKinematicsDerivatives(robot.get_robot_model(), robot.get_robot_data(), q_propagated[i], v_propagated[i], v_dot_eq);
        
        robot.get_swing_acceleration_derivatives(da_dq, da_dv, swing_feet_names, swing_feet_index);

        auto qw = q_propagated[i](6);
        auto qx = q_propagated[i](3);
        auto qy = q_propagated[i](4);
        auto qz = q_propagated[i](5);

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 3);
        Q.row(0) << qw, -qz, qy;
        Q.row(1) << qz, qw, -qx;
        Q.row(2) << -qy, qx, qw;
        Q.row(3) << -qx, -qy, -qz;

        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(q_dim, v_dim);
        G.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3,3);                                           //         [p_b_dot]   [  v_b  ]   [I 0 0] [  v_b  ] 
        G.bottomRightCorner(joint_dim, joint_dim) = Eigen::MatrixXd::Identity(joint_dim, joint_dim);      // q_dot = [q_b_dot] = [ Q*w_b ] = [0 Q 0] [  w_b  ]
        G.block(3, 3, 4, 3) = 0.5*Q;                                                                      //         [q_j_dot]   [q_j_dot]   [0 0 I] [q_j_dot]

        da_dq = da_dq.eval()*G.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::MatrixXd df_du = Eigen::MatrixXd::Zero(12, joint_dim+contact_dim);
        df_du.leftCols(joint_dim) = Js*robot.get_robot_data().Minv*robot.get_S().transpose();
        df_du.rightCols(contact_dim) = Js*robot.get_robot_data().Minv*(Jc_in_time[i].transpose());

        Eigen::VectorXd uc = Eigen::VectorXd::Zero(joint_dim+contact_dim);
        uc.head(joint_dim) = robot.get_optimal_torques()[i];
        uc.tail(contact_dim) = robot.get_optimal_contact_forces()[i];

        Eigen::VectorXd f_xc = Eigen::VectorXd::Zero(12);
        Eigen::MatrixXd K1 = Eigen::MatrixXd::Zero(12, 12);
        Eigen::MatrixXd K2 = Eigen::MatrixXd::Zero(12, 12);

        for(int j=0; j<swing_feet_names.size(); j++){
            pinocchio::FrameIndex foot_id = robot.get_robot_model().getFrameId(swing_feet_names[j]);
            Eigen::VectorXd swing_foot_vel = pinocchio::getFrameVelocity(robot.get_robot_model(), robot.get_robot_data(), foot_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
            Eigen::VectorXd foot_pos = robot.get_robot_data().oMf[foot_id].translation();

            K1.block(3*swing_feet_index[j], 3*swing_feet_index[j], 3, 3) = Kp_s_pos;
            K2.block(3*swing_feet_index[j], 3*swing_feet_index[j], 3, 3) = Kd_s_pos;

            f_xc.segment(swing_feet_index[j]*3, 3) = Js_dot_times_v.segment(3*swing_feet_index[j], 3)
                - gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.segment(j*3, 3)
                - Kd_s_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.segment(j*3, 3) - feet_vel.segment(3*swing_feet_index[j], 3))
                - Kp_s_pos.asDiagonal()*(gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.segment(j*3, 3) - foot_pos.topRows(3));
        }

        
        A.block(i*12, (i-1)*(q_dim+v_dim), 12, q_dim) = da_dq + K1*Js;    // df_dq = da_dq - Kp*Jb
        A.block(i*12, (i-1)*(q_dim+v_dim)+q_dim, 12, v_dim) = da_dv + K2*Js_dot;
        A.block(i*12, mpc_step_horizon*(q_dim+v_dim) + i*(joint_dim+contact_dim), 12, joint_dim+contact_dim) = df_du;
        b.segment(i*12, 12) = (da_dq+K1*Js)*q_propagated[i] + (da_dv+K2*Js_dot)*v_propagated[i] + df_du*uc - f_xc;
        

       // ############################################################################################################
    }
    Task motion_tracking_swing_feet(A, b, D, f);
    //std::cout <<"Task motion tracking feet:\n" <<motion_tracking_swing_feet <<std::endl;
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

    Eigen::MatrixXd A = MatrixXd::Zero(3*ncp*mpc_step_horizon, mpc_step_horizon*cols);
    Eigen::VectorXd b = VectorXd::Zero(3*ncp*mpc_step_horizon);
    //Eigen::MatrixXd A = Eigen::MatrixXd::Zero(0, mpc_step_horizon*cols);
    //Eigen::VectorXd b = Eigen::VectorXd::Zero(0);
    Eigen::MatrixXd D = MatrixXd::Zero(6*ncp*mpc_step_horizon, cols*mpc_step_horizon);
    Eigen::VectorXd f = VectorXd::Zero(6*ncp*mpc_step_horizon);

    if (gen_poses.generalized_poses_with_time[0].gen_pose.contact_feet_names.size() == 4){
        A.resize(0, NoChange);
        b.resize(0, NoChange);
    }

    for (int i = 0; i < mpc_step_horizon; i++) {
        std::vector<std::string> contact_feet_names = gen_poses.generalized_poses_with_time[i].gen_pose.contact_feet_names;

        // Foot name conversion
        std::vector<std::string> robot_short_feet_names = robot.get_short_feet_names();
        std::vector<std::string> robot_feet_names = robot.get_feet_names();
        std::vector<int> contact_feet_index(0);
        for (auto & foot_name : contact_feet_names) {
            auto it = std::find(robot_short_feet_names.begin(), robot_short_feet_names.end(), foot_name);
            int index = it - robot_short_feet_names.begin();
            contact_feet_index.push_back(index);
            foot_name = robot_feet_names[index];
        }


        std::vector<int> swing_feet_index(0);
        for (size_t j= 0; j < 4; j++) {
            if ( std::find(contact_feet_names.begin(), contact_feet_names.end(), robot_feet_names[j]) == contact_feet_names.end() ) {
                // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing foot.
                swing_feet_index.push_back(j);
            }
        }

        D.block(i*(6*ncp), mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = h-mu*n;
        D.block(i*(6*ncp)+ncp, mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = -h-mu*n;
        D.block(i*(6*ncp)+2*ncp, mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = l-mu*n;
        D.block(i*(6*ncp)+3*ncp, mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = -l-mu*n;
        D.block(i*(6*ncp)+4*ncp, mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = n;
        D.block(i*(6*ncp)+5*ncp, mpc_step_horizon*(q_dim+v_dim) + joint_dim + i*(joint_dim+contact_forces_dim), ncp, contact_forces_dim) = -n;

        f.segment(i*(6*ncp) + 4*ncp, ncp) = Eigen::VectorXd::Ones(ncp)*robot.get_f_max();
        f.segment(i*(6*ncp) + 5*ncp, ncp) = -Eigen::VectorXd::Ones(ncp)*robot.get_f_min();

        for (int j=0; j<swing_feet_index.size(); j++){
            D.row(i*(6*ncp) + swing_feet_index[j]).setZero();
            D.row(i*(6*ncp) + swing_feet_index[j] + ncp).setZero();
            D.row(i*(6*ncp) + swing_feet_index[j] + 2*ncp).setZero();
            D.row(i*(6*ncp) + swing_feet_index[j] + 3*ncp).setZero();
            D.row(i*(6*ncp) + swing_feet_index[j] + 4*ncp).setZero();
            D.row(i*(6*ncp) + swing_feet_index[j] + 5*ncp).setZero();

            f(i*(6*ncp) + swing_feet_index[j] + 4*ncp) = 0;
            f(i*(6*ncp) + swing_feet_index[j] + 5*ncp) = 0;

            A.block(i*(6*ncp) + swing_feet_index[j]*3, mpc_step_horizon*(q_dim+v_dim)+ joint_dim + i*(joint_dim+contact_forces_dim)+swing_feet_index[j]*3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
        }
    }

    Task friction(A, b, D, f);
    return friction;
}

Task MPC::effort_minimization(){
    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
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
    //std::cout <<"Effort minimization:\n" <<effort_minimization <<std::endl;
    return effort_minimization;
}


std::vector<Eigen::VectorXd> MPC::tune_gains(Eigen::VectorXd q_, Eigen::VectorXd v_, GeneralizedPosesWithTime gen_poses){

    int q_dim = robot.get_robot_model().nq;
    int v_dim = robot.get_robot_model().nv;
    int joint_dim = v_dim - 6;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(q_dim);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(v_dim);

    std::vector<Eigen::VectorXd> Kp(number_of_wolves);
    std::vector<Eigen::VectorXd> Kd(number_of_wolves);
    std::vector<Eigen::VectorXd> Ki(number_of_wolves);
    for (int i=0; i<number_of_wolves; i++){
        Kp[i] = Eigen::VectorXd::Random(joint_dim)*100;
        Kd[i] = Eigen::VectorXd::Random(joint_dim)*100;
        Ki[i] = Eigen::VectorXd::Random(joint_dim)*100;
        Kp[i].cwiseAbs();
        Kd[i].cwiseAbs();
        Ki[i].cwiseAbs();
    }

    q = q_;
    v = v_;

    Eigen::VectorXd fit_f = VectorXd::Zero(number_of_wolves);


    for (int i=0; i<number_of_wolves; i++){
        // Compute error
        Eigen::VectorXd error = VectorXd::Zero(2*joint_dim);
        Eigen::VectorXd error_prev = VectorXd::Zero(2*joint_dim);
        Eigen::VectorXd error_int = VectorXd::Zero(2*joint_dim);

        // gen_poses contains the desired pose, not the desired joint pos and vel
        // TODO keep in consideration this
        // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        Eigen::VectorXd q_opt = robot.get_optimal_q()[0];
        Eigen::VectorXd v_opt = robot.get_optimal_v()[0];
        error.head(joint_dim) = q.tail(joint_dim) - q_opt.tail(joint_dim);
        error.tail(joint_dim) = v.tail(joint_dim) - v_opt.tail(joint_dim);

        // std::cout <<"Errore:\n" << error <<"\n";

        fit_f(i) = fit_f(i) + error.transpose()*error;
        // std::cout <<"Fit function:\n" <<fit_f <<"\n";
        std::vector<Eigen::VectorXd> state_propagated(2);

        // Propagate dynamics
        std::vector<Eigen::VectorXd> joint_state_propagated(2);
        Eigen::VectorXd tau = VectorXd::Zero(v_dim);
        for (int j=1; j<mpc_step_horizon; j++){
            tau.tail(joint_dim) = Kp[i]*error + Kd[i]*(error-error_prev)/dT + Ki[i]*error_int;
            // std::cout <<"Tau PID\n" <<tau <<"\n";
            state_propagated = robot.compute_dynamics(q, v, tau, Eigen::VectorXd::Zero(12), dT);
            q = state_propagated[0];
            v = state_propagated[1];

            q_opt = robot.get_optimal_q()[j];
            v_opt = robot.get_optimal_v()[j];

        // Error computation
            error_prev = error;
            error.head(joint_dim) = q.tail(joint_dim) - q_opt.tail(joint_dim);
            error.tail(joint_dim) = v.tail(joint_dim) - v_opt.tail(joint_dim);
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

Eigen::VectorXd MPC::get_feet_positions(){
    Eigen::VectorXd feet_position(12);
    std::vector<std::string> feet_names = robot.get_feet_names();

    for (size_t i = 0; i < feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = robot.get_robot_model().getFrameId(feet_names[i]);

        feet_position.segment(3*i, 3) = robot.get_robot_data().oMf[frame_id].translation();
    }

    return feet_position;
}

Eigen::VectorXd MPC::get_feet_velocities(Eigen::VectorXd v){
    Eigen::VectorXd feet_velocities(12);
    std::vector<std::string> feet_names = robot.get_feet_names();
    Eigen::MatrixXd J_temp(6, robot.get_robot_model().nv);

    for (size_t i = 0; i < feet_names.size(); i++) {
        J_temp.setZero();
        pinocchio::FrameIndex frame_id = robot.get_robot_model().getFrameId(feet_names[i]);

        pinocchio::getFrameJacobian(robot.get_robot_model(), robot.get_robot_data(), frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        feet_velocities.segment(0+3*i, 3) = J_temp.topRows(3) * v;
    }

    return feet_velocities;
}

