#include "model_predictive_control/Robot.hpp"

using namespace Eigen;

Robot::Robot(){}

Robot::Robot(std::string robot_name){
    this->robot_name = robot_name;

    if(robot_name == "solo12"){
      std::cout <<"Robot name: " <<robot_name <<std::endl;
      const std::string urdf_filename = std::string("src/robot/robots/solo_description/urdf/solo12.urdf");
      pinocchio::JointModelFreeFlyer root_joint;
      pinocchio::urdf::buildModel(urdf_filename, root_joint, this->robot_model);
    }
    else{
      std::cout <<"Robot name: " <<robot_name <<std::endl;
      const std::string urdf_filename = std::string("src/robot/robots/anymal_c_simple_description/urdf/anymal.xacro");
      pinocchio::JointModelFreeFlyer root_joint;
      pinocchio::urdf::buildModel(urdf_filename, root_joint, this->robot_model);
    }

    this->robot_data = pinocchio::Data(robot_model);    // Create data required by the algorithms
    this->state_dim = this->robot_model.nv;
    this->contact_feet_dim = 4;
    if(robot_name == "anymal_c"){
      this->feet_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    }

    S = Eigen::MatrixXd::Zero(n_act, robot_model.nv);
    S.rightCols(n_act) = Eigen::MatrixXd::Identity(n_act, n_act);

    this->dT = 1.0/400;

}

void Robot::compute_EOM(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
  //std::cout <<"Stampa prima forward Kin\n" <<std::endl;
    // Update the joint placements
    pinocchio::forwardKinematics(robot_model, robot_data, q);

  //std::cout <<"Stampa prima computeJointJac\n" <<std::endl;
    // Computes the full model Jacobian
    pinocchio::computeJointJacobians(robot_model, robot_data, q);

  //std::cout <<"Stampa prima updateFramePlac\n" <<std::endl;
    // Update the frame placements
    pinocchio::updateFramePlacements(robot_model, robot_data);

  //std::cout <<"Stampa prima crba\n" <<std::endl;
    // Compute the upper part of the joint space inertia matrix
    pinocchio::crba(robot_model, robot_data, q);
    robot_data.M.triangularView<Eigen::StrictlyLower>() = robot_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the nonlinear effects vector (Coriolis, centrifugal and gravitational effects)
    pinocchio::nonLinearEffects(robot_model, robot_data, q, v);
}

std::vector<Eigen::VectorXd> Robot::compute_dynamics(Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd tau, Eigen::VectorXd f_ext, double dT){

    Eigen::VectorXd v_dot = Eigen::VectorXd::Zero(robot_model.nv);
    v_dot = robot_data.Minv*(tau + f_ext - robot_data.nle);
  
  /*
    Eigen::Vector3d f1_e = f_ext.segment(0, 3);
    pinocchio::Force f1;
    f1.linear(f1_e);
    Eigen::Vector3d f2_e = f_ext.segment(3, 3);
    pinocchio::Force f2;
    f2.linear(f2_e);
    Eigen::Vector3d f3_e = f_ext.segment(6, 3);
    pinocchio::Force f3;
    f3.linear(f3_e);
    Eigen::Vector3d f4_e = f_ext.segment(9, 3);
    pinocchio::Force f4;
    f4.linear(f4_e);
    pinocchio::container::aligned_vector<pinocchio::Force> f;
    f.push_back(f1);
    f.push_back(f2);
    f.push_back(f3);
    f.push_back(f4);
    */
    pinocchio::aba(this->robot_model, this->robot_data, q, v, tau);
    Eigen::VectorXd q_ddot = robot_data.ddq;
    q_ddot = q_ddot.eval() + robot_data.Minv*f_ext;
    //std::cout <<"q_ddot in compute dynamics:\n" <<q_ddot <<std::endl;
    
    //std::cout <<"v_dot in compute dynamics:\n" <<v_dot <<std::endl;
    Eigen::VectorXd v_updated = v + dT*q_ddot;
    //std::cout <<"v_updated:\n" <<v_updated <<std::endl;
    Eigen::VectorXd q_updated = pinocchio::integrate(this->robot_model, q, v_updated);
    //std::cout <<"q_updated:\n" <<q_updated <<std::endl;
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    joint_state_propagated[0] = q_updated;
    joint_state_propagated[1] = v_updated;
    return joint_state_propagated;
}

 void Robot::compute_second_order_FK(const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd v_dot)
 {
     // Update the joint accelerations
     pinocchio::forwardKinematics(this->robot_model, this->robot_data, q, v, Eigen::VectorXd::Zero(robot_model.nv));

     // Computes the full model Jacobian variations with respect to time
     pinocchio::computeJointJacobiansTimeVariation(this->robot_model, this->robot_data, q, v);
 }

void Robot::get_Jc(Eigen::MatrixXd& Jc, std::vector<std::string> ground_feet_names, std::vector<int> contact_feet_index)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.

    //pinocchio::computeJointJacobians(robot_model, robot_data, q);
    //pinocchio::framesForwardKinematics(robot_model, robot_data, q);

    Eigen::MatrixXd J_temp = MatrixXd::Zero(6, robot_model.nv);

    // Compute the stack of the contact Jacobians
    for (int i = 0; i < ground_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(ground_feet_names[i]);
        //std::cout <<"feet name dentro get_Jc: " <<ground_feet_names[i] <<std::endl;
        //std::cout <<"Frame id dentro get_Jc: " <<frame_id <<std::endl;
        //std::cout <<"contact feet index dentro get_Jc: " <<contact_feet_index[i] <<std::endl;
        J_temp.setZero();
        pinocchio::getFrameJacobian(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);
        //std::cout <<"contact_feet_index[i]: " <<contact_feet_index[i] <<std::endl;
        Jc.block(3*contact_feet_index[i], 0, 3, robot_model.nv) = J_temp.topRows(3);
    }
}

void Robot::get_Jc_dot(Eigen::MatrixXd& Jc_dot, std::vector<std::string> ground_feet_names, std::vector<int> contact_feet_index)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.

    Eigen::MatrixXd J_temp = MatrixXd::Zero(6, robot_model.nv);

    // Compute the stack of the contact Jacobians
    for (int i = 0; i < ground_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(ground_feet_names[i]);
        J_temp.setZero();
        pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);
        Jc_dot.block(3*contact_feet_index[i], 0, 3, robot_model.nv) = J_temp.topRows(3);
    }
}

void Robot::get_Jc_dot_times_v(Eigen::VectorXd& Jc_dot_times_v, std::vector<std::string> contact_feet_names, std::vector<int> contact_feet_index){
  for (int i=0; i<contact_feet_names.size(); i++){
      pinocchio::FrameIndex frame_id = robot_model.getFrameId(contact_feet_names[i]);
  Jc_dot_times_v.segment(0+3*contact_feet_index[i], 3) = pinocchio::getFrameClassicalAcceleration(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
  }
  //std::cout <<"Jc_dot_times_v dentro get ecc ecc :\n" <<Jc_dot_times_v <<std::endl;
}

void Robot::get_Jb(Eigen::MatrixXd& Jb)
{
    Jb.setZero();

    pinocchio::FrameIndex base_id = 1;

    pinocchio::getFrameJacobian(robot_model, robot_data, base_id, pinocchio::LOCAL_WORLD_ALIGNED, Jb);
}

void Robot::get_Jb_dot(Eigen::MatrixXd& Jb_dot){
  Jb_dot.setZero();

  //pinocchio::FrameIndex base_id = robot_model.getFrameId("base");
  pinocchio::FrameIndex base_id = 1;

  pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, base_id, pinocchio::LOCAL_WORLD_ALIGNED, Jb_dot);
}

void Robot::get_Jb_dot_times_v(Eigen::VectorXd& Jb_dot_times_v){
  pinocchio::FrameIndex base_id = 1;

  Jb_dot_times_v = pinocchio::getClassicalAcceleration(robot_model, robot_data, base_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
}

void Robot::get_Js(Eigen::MatrixXd& Js, std::vector<std::string> swing_feet_names, std::vector<int> swing_feet_index)
{
    // Initialize a temp Jacobian that must be used to store the swing feet jacobian.
    // Js is the stack of J_temp of all the swing feet.
    Eigen::MatrixXd J_temp(6, robot_model.nv);

    if(!swing_feet_names.empty()){
      // Compute the stack of the swing jacobians.
      for (size_t i = 0; i < swing_feet_names.size(); i++) {
          pinocchio::FrameIndex frame_id = robot_model.getFrameId(swing_feet_names[i]);
          //std::cout <<"frame_id dentro get_Js: " <<frame_id <<std::endl;

          J_temp.setZero();
          
          pinocchio::getFrameJacobian(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

          // swing_feet_index contains the indexes of the swing feet in the "feet_names" vector ex. index=0 => FL_FOOT
          int index = swing_feet_index[i];
          //std::cout <<"Index dentro get_Js: " <<index <<std::endl;

          Js.block(3*index, 0, 3, robot_model.nv) = J_temp.topRows(3);
          //Js.block(3*index, 6, 3, 12) = J_temp.block(0, 6, 3, 12);
      }
    }
}

void Robot::get_Js_dot(Eigen::MatrixXd& Js_dot, std::vector<std::string> swing_feet_names, std::vector<int> swing_feet_index)
{
    // Initialize a temp Jacobian that must be used to store the swing feet jacobian.
    // Js is the stack of J_temp of all the swing feet.
    Eigen::MatrixXd J_temp(6, robot_model.nv);

    if(!swing_feet_names.empty()){
      // Compute the stack of the swing jacobians.
      for (size_t i = 0; i < swing_feet_names.size(); i++) {
          pinocchio::FrameIndex frame_id = robot_model.getFrameId(swing_feet_names[i]);
          //std::cout <<"frame_id dentro get_Js: " <<frame_id <<std::endl;

          J_temp.setZero();
          
          pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

          // swing_feet_index contains the indexes of the swing feet in the "feet_names" vector ex. index=0 => FL_FOOT
          int index = swing_feet_index[i];
          //std::cout <<"Index dentro get_Js: " <<index <<std::endl;

          Js_dot.block(3*index, 0, 3, robot_model.nv) = J_temp.topRows(3);
          //Js.block(3*index, 6, 3, 12) = J_temp.block(0, 6, 3, 12);
      }
    }
}

void Robot::get_Js_dot_times_v(Eigen::VectorXd& Js_dot_times_v, std::vector<std::string> swing_feet_names, std::vector<int> swing_feet_index)
{

  if(!swing_feet_names.empty()){
      // Compute the stack of the swing jacobians.
      for (size_t i = 0; i < swing_feet_names.size(); i++) {
          pinocchio::FrameIndex frame_id = robot_model.getFrameId(swing_feet_names[i]);
          
          Js_dot_times_v.segment(0+3*swing_feet_index[i], 3) = pinocchio::getFrameClassicalAcceleration(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
      }
    }
}


void Robot::get_Jfull(Eigen::MatrixXd& J){
  Eigen::MatrixXd J_temp(6, robot_model.nv);

  for(size_t i=0; i<feet_names.size(); i++){
    pinocchio::FrameIndex frame_id = robot_model.getFrameId(feet_names[i]);

    J_temp.setZero();

    pinocchio::getFrameJacobian(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);
    J.block(3*i, 0, 3, robot_model.nv) = J_temp.topRows(3);
  }
}

void Robot::compute_terms(Eigen::VectorXd q, Eigen::VectorXd v){
    // Update the joint placements
    pinocchio::forwardKinematics(robot_model, robot_data, q);

    // Computes the full model Jacobian
    pinocchio::computeJointJacobians(robot_model, robot_data, q);

    // Update the frame placements
    pinocchio::updateFramePlacements(robot_model, robot_data);

    // Compute the upper part of the joint space inertia matrix
    pinocchio::crba(robot_model, robot_data, q);
    robot_data.M.triangularView<Eigen::StrictlyLower>() = robot_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the nonlinear effects vector (Coriolis, centrifugal and gravitational effects)
    pinocchio::nonLinearEffects(robot_model, robot_data, q, v);
}

void Robot::get_contact_acceleration_derivatives(Eigen::MatrixXd& da_dq, Eigen::MatrixXd& da_dv, std::vector<std::string> contact_feet_names, std::vector<int> contact_feet_index){
    for(int i=0; i<contact_feet_names.size(); i++){
      Eigen::MatrixXd dv_dq = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd dv_dv = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd da_da = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd partial_a_dq = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd partial_a_dv = Eigen::MatrixXd::Zero(6, robot_model.nv);

      pinocchio::FrameIndex frame_id = robot_model.getFrameId(contact_feet_names[i]); 
      pinocchio::getFrameAccelerationDerivatives(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dv_dq, partial_a_dq, partial_a_dv, da_da);
      da_dq.block(3*contact_feet_index[i], 0, 3, robot_model.nv) = partial_a_dq.topRows(3);
      da_dv.block(3*contact_feet_index[i], 0, 3, robot_model.nv) = partial_a_dv.topRows(3);
    }
}

void Robot::get_base_acceleration_derivatives(Eigen::MatrixXd& da_dq, Eigen::MatrixXd& da_dv){
    pinocchio::FrameIndex base_id = 1;

      Eigen::MatrixXd dv_dq = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd dv_dv = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd da_da = Eigen::MatrixXd::Zero(6, robot_model.nv);
      pinocchio::getFrameAccelerationDerivatives(robot_model, robot_data, base_id, pinocchio::LOCAL_WORLD_ALIGNED, dv_dq, da_dq, da_dv, da_da);
}

void Robot::get_swing_acceleration_derivatives(Eigen::MatrixXd& da_dq, Eigen::MatrixXd& da_dv, std::vector<std::string> swing_feet_names, std::vector<int> swing_feet_index){
    for(int i=0; i<swing_feet_names.size(); i++){
      Eigen::MatrixXd dv_dq = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd dv_dv = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd da_da = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd partial_a_dq = Eigen::MatrixXd::Zero(6, robot_model.nv);
      Eigen::MatrixXd partial_a_dv = Eigen::MatrixXd::Zero(6, robot_model.nv);

      pinocchio::FrameIndex frame_id = robot_model.getFrameId(swing_feet_names[i]); 
      pinocchio::getFrameAccelerationDerivatives(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dv_dq, partial_a_dq, partial_a_dv, da_da);
      da_dq.block(3*swing_feet_index[i], 0, 3, robot_model.nv) = partial_a_dq.topRows(3);
      da_dv.block(3*swing_feet_index[i], 0, 3, robot_model.nv) = partial_a_dv.topRows(3);
    }
}

/*
Eigen::VectorXd Robot::clik_alg(Eigen::VectorXd q_act, GeneralizedPose des_pose, pinocchio::Model model, pinocchio::Data data){
    //const int JOINT_ID = 6;
   //const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));

   std::cout <<"Stampa 1\n" <<std::endl;

   Eigen::Quaterniond quat;
   quat.x() = des_pose.base_quat(0);
   quat.y() = des_pose.base_quat(1);
   quat.z() = des_pose.base_quat(2);
   quat.w() = des_pose.base_quat(3);
   Eigen::Matrix3d rot_mat = quat.toRotationMatrix();

   pinocchio::SE3 oMdes(rot_mat, des_pose.base_pos);
   std::cout <<"Stampa 2\n" <<std::endl;
   
   //const int JOINT_ID = model.getFrameId("base_link");
   const int JOINT_ID = 1;
   int pippo_id = model.getFrameId("pippo");
   std::cout <<"Stampa 3, JOINT_ID= " <<JOINT_ID <<"\n" <<std::endl;
   std::cout <<"Stampa pippo joint_id= " <<pippo_id <<"\n" <<std::endl;
    //const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));
  
   Eigen::VectorXd q = pinocchio::neutral(model);
   const double eps  = 1e-4;
   const int IT_MAX  = 1000;
   const double DT   = 1e-1;
   const double damp = 1e-6;
  
   pinocchio::Data::Matrix6x J(6,model.nv);
   J.setZero();
   std::cout <<"Stampa 4\n" <<std::endl;
  
   bool success = false;
   typedef Eigen::Matrix<double, 6, 1> Vector6d;
   Vector6d err;
   Eigen::VectorXd v(model.nv);
   for (int i=0;;i++)
   {
    std::cout <<"Stampa 5\n" <<std::endl;
     pinocchio::forwardKinematics(model,data,q);
     std::cout <<"Stampa 6\n" <<std::endl;
     const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
     std::cout <<"Stampa 7\n" <<std::endl;
     err = pinocchio::log6(dMi).toVector();
     std::cout <<"Stampa 8\n" <<std::endl;
     if(err.norm() < eps)
     {
       success = true;
       break;
     }
     if (i >= IT_MAX)
     {
       success = false;
       break;
     }
     std::cout <<"Stampa 9\n" <<std::endl;
     pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);
     std::cout <<"Stampa 10\n" <<std::endl;
     pinocchio::Data::Matrix6 JJt;
     std::cout <<"Stampa 11\n" <<std::endl;
     JJt.noalias() = J * J.transpose();
     std::cout <<"Stampa 12\n" <<std::endl;
     JJt.diagonal().array() += damp;
     std::cout <<"Stampa 13\n" <<std::endl;
     v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
     std::cout <<"Stampa 14\n" <<std::endl;
     q = pinocchio::integrate(model,q,v*DT);
     std::cout <<"Stampa 15\n" <<std::endl;
     if(!(i%10))
       std::cout << i << ": error = " << err.transpose() << std::endl;
   }
  
   if(success) 
   {
     std::cout << "Convergence achieved!" << std::endl;
   }
   else 
   {
     std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
   }
     
   std::cout << "\nresult: " << q.transpose() << std::endl;
   std::cout << "\nfinal error: " << err.transpose() << std::endl;
   return q;
}
*/

/*
  ########################################################################################
                        PROVA DI IMPLEMENTAZIONE CINEMATICA INVERSA
  ########################################################################################

            [ p_base ]            [   v_base   ]
    q_gen = [ q_base ]    u_gen = [   w_base   ]  v_base, w_base from planner
            [q_joints]            [q_dot_joints]

            for swinging feet: J_fi * u_gen = v_swing
            for contact feet: J_fi * u_gen = 0

  Eigen::VectorXd inverse_kinematics(GeneralizedPose gen_pose, pinocchio::Model model){
      std::vector<std::string> contact_feet_names = gen_pose.contact_feet;

      Eigen::MatrixXd J_temp(6, model.nv);
      std::vector<Eigen::VectorXd> q_dot
      for (size_t i = 0; i < contact_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(contact_feet_names[i]);

        J_temp.setZero();
        
        pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        Eigen::VectorXd b = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd q_dot_j = J_temp.colPivHouseholderQr().solve(b);

        
    }
  }
*/
