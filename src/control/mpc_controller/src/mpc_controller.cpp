#include "mpc_controller/mpc_controller.hpp"
#include "mpc_controller/mpc_publisher.hpp"
#include <chrono>>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mpc_controller{

using namespace std::chrono;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

    MPCController::MPCController():mpc("solo12", 1.0/400, mpc_step_horizon){
        std::cout <<"\n\n****************** Controller created ****************** \n\n" <<std::endl;
        }
    
    CallbackReturn MPCController::on_init(){
    std::cout <<"\n\n******************** Starting on init ********************\n\n" <<std::endl;
        try{
            auto_declare<std::string>("robot_name", std::string());
            auto_declare<double>("sample_time", double());
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<bool>("use_estimator", bool());

            auto_declare<std::vector<double>>("q_init", std::vector<double>());

            auto_declare<bool>("logging", bool());
            auto_declare<double>("tau_max", double());
            auto_declare<double>("tau_min", double());
            auto_declare<double>("f_max", double());
            auto_declare<double>("f_min", double());
            auto_declare<double>("mu", double());

            auto_declare<std::vector<double>>("Kp_pos", std::vector<double>());
            auto_declare<std::vector<double>>("Kd_pos", std::vector<double>());
            auto_declare<std::vector<double>>("Kp_ang", std::vector<double>());
            auto_declare<std::vector<double>>("Kd_ang", std::vector<double>());
            auto_declare<std::vector<double>>("Kp_s_pos", std::vector<double>());
            auto_declare<std::vector<double>>("Kd_s_pos", std::vector<double>());
            auto_declare<std::vector<double>>("Kp_terr", std::vector<double>());

            // auto_declare<double>("reg", double());

        }
        catch(const std::exception& e) {
            fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        std::cout <<"\n\n******************** Ending on init ********************\n\n" <<std::endl;
        return CallbackReturn::SUCCESS;
    }

    /* ===================== Command_interface_configuration ==================== */

    InterfaceConfiguration MPCController::command_interface_configuration() const{
        InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : this->joint_names) {
            command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
        }

        return command_interfaces_config;
    }


    /* ====================== State_interface_configuration ===================== */

    InterfaceConfiguration MPCController::state_interface_configuration() const{
        InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : this->joint_names) {
            state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
            state_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
        }
        
        return state_interfaces_config;
    }

    CallbackReturn MPCController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
    
    std::cout <<"\n\n******************** Starting on configure ********************\n\n";
    
        const std::string robot_name = get_node()->get_parameter("robot_name").as_string();     //controller_interface_base::get_node() returns LifecycleNode
        if (robot_name.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "robot_name parameter is empty");
            return CallbackReturn::ERROR;
        }

        const double dT = get_node()->get_parameter("sample_time").as_double();
        if(dT <= 0){
            RCLCPP_ERROR(get_node()->get_logger(), "sample time parameter is less or equal 0");
            return CallbackReturn::ERROR;
        }

        this->joint_names = get_node()->get_parameter("joints").as_string_array();
        if (this->joint_names.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "joints is empty");
            return CallbackReturn::ERROR;
        }
        for(int i=0; i<joint_names.size(); i++){
            std::cout <<"joint_names:\n" <<joint_names[i] <<std::endl;
        }

        const bool use_estimator = get_node()->get_parameter("use_estimator").as_bool();

        auto_declare<double>("initialization_time", double());

        this->logging = get_node()->get_parameter("logging").as_bool();
        
        auto q_i = get_node()->get_parameter("q_init").as_double_array();
        if(q_i.size() != 12){
            RCLCPP_ERROR(get_node()->get_logger(), "q_init does not have 12 elements");
            return CallbackReturn::ERROR;
        }
        this->q_init = Eigen::VectorXd::Map(q_i.data(), q_i.size());
        std::cout <<"q_init configure:\n" <<this->q_init <<"\n" <<std::endl;

        if(get_node()->get_parameter("mpc_step_horizon").as_int() <= 0){
            RCLCPP_ERROR(get_node()->get_logger(), "'mpc_step_horizon' is <= 0");
            return CallbackReturn::ERROR;
        }
        this->mpc_step_horizon = get_node()->get_parameter("mpc_step_horizon").as_int();

        //std::cout <<"mpc_steps: " <<mpc_step_horizon <<std::endl;
        mpc = MPC(robot_name, dT, mpc_step_horizon);

        this->q.resize(mpc.get_robot().get_state_dim() + 1);
        q(6) = 1;
        this->v.resize(mpc.get_robot().get_state_dim());

        for (int i=0; i<des_gen_poses.generalized_poses_with_time.size(); i++) {
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.resize(3);
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.resize(3);
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.resize(3);
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_pos.resize(0);
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_vel.resize(0);
            des_gen_poses.generalized_poses_with_time[i].gen_pose.feet_acc.resize(0);

            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_pos.resize(3);
            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_vel.resize(3);
            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_acc.resize(3);
            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_pos.resize(0);
            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_vel.resize(0);
            des_gen_poses_single.generalized_poses_with_time[i].gen_pose.feet_acc.resize(0);
        }


        double tau_max = get_node()->get_parameter("tau_max").as_double();
        if (get_node()->get_parameter("tau_max").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'tau_max' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.set_tau_max(tau_max);
        std::cout <<"tau_max robot: " <<mpc.get_robot().get_tau_max() <<"\n" <<std::endl;

        if (get_node()->get_parameter("tau_min").as_double() <= -100.) {
            RCLCPP_ERROR(get_node()->get_logger(),"'tau_min' parameter is too low");
            return CallbackReturn::ERROR;
        }
        mpc.set_tau_min(get_node()->get_parameter("tau_min").as_double());
        std::cout <<"tau_min robot: " <<mpc.get_robot().get_tau_min() <<"\n" <<std::endl;
    
        if (get_node()->get_parameter("mu").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'mu' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.set_mu(get_node()->get_parameter("mu").as_double());
        
        if (get_node()->get_parameter("f_max").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'f_max' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.set_f_max(get_node()->get_parameter("f_max").as_double());
        std::cout <<"f_max robot: " <<mpc.get_robot().get_f_max() <<"\n" <<std::endl;
        
        if (get_node()->get_parameter("f_min").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'f_min' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.set_f_min(get_node()->get_parameter("f_min").as_double());
        std::cout <<"f_min robot: " <<mpc.get_robot().get_f_min() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kp_PD").as_double() <= 0){
            RCLCPP_ERROR(get_node()->get_logger(), "'PD gain Kp is <= 0");
            return CallbackReturn::ERROR;
        }
        this->Kp_PD = get_node()->get_parameter("Kp_PD").as_double();

        if (get_node()->get_parameter("Kd_PD").as_double() <= 0){
            RCLCPP_ERROR(get_node()->get_logger(), "'PD gain Kd is <= 0");
            return CallbackReturn::ERROR;
        }
        this->Kd_PD = get_node()->get_parameter("Kd_PD").as_double(); 

        if (get_node()->get_parameter("Kp_pos").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kp_pos' parameter is does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kp_pos(Eigen::Vector3d::Map(get_node()->get_parameter("Kp_pos").as_double_array().data()));
        std::cout <<"Kp_pos robot: " <<mpc.get_Kp_pos() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kd_pos").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kd_pos' parameter does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kd_pos(Eigen::Vector3d::Map(get_node()->get_parameter("Kd_pos").as_double_array().data()));
        std::cout <<"Kd_pos robot: " <<mpc.get_Kd_pos() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kp_ang").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kp_ang' parameter does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kp_ang(Eigen::Vector3d::Map(get_node()->get_parameter("Kp_ang").as_double_array().data()));
        std::cout <<"Kp_ang robot: " <<mpc.get_Kp_ang() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kd_ang").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kd_ang' parameter does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kd_ang(Eigen::Vector3d::Map(get_node()->get_parameter("Kd_ang").as_double_array().data()));
        std::cout <<"Kd_ang robot: " <<mpc.get_Kd_ang() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kp_s_pos").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kp_s_pos' parameter does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kp_s_pos(Eigen::Vector3d::Map(get_node()->get_parameter("Kp_s_pos").as_double_array().data()));
        std::cout <<"Kp_s_pos robot: " <<mpc.get_Kp_s_pos() <<"\n" <<std::endl;

        if (get_node()->get_parameter("Kd_s_pos").as_double_array().size() != 3){
            RCLCPP_ERROR(get_node()->get_logger(), "'Kd_s_pos' parameter does not have 3 elements");
            return CallbackReturn::ERROR;
        }
        mpc.set_Kd_s_pos(Eigen::Vector3d::Map(get_node()->get_parameter("Kd_s_pos").as_double_array().data()));
        std::cout <<"Kd_s_pos robot: " <<mpc.get_Kd_s_pos() <<"\n" <<std::endl;



        if (!use_estimator){
            // Use the simulated data of Gazebo
            std::cout <<"Using Gazebo data" <<std::endl;

            this->joint_state_subscription = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
                "/gazebo/link_states", 1,
                [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg)->void{
                    int base_id = -1;

                    for(std::size_t i=0; i<msg->name.size(); i++){
                        if(msg->name[i].size() >= 4){
                            if(msg->name[i].find("base") != std::string::npos){
                                base_id = i;
                                break;
                            }
                        }
                    }

                    if (base_id == -1){
                        RCLCPP_ERROR(get_node()->get_logger(), "Can't find a link name containing 'base'");
                    }

                    const geometry_msgs::msg::Point pos = msg->pose[base_id].position;
                    const geometry_msgs::msg::Quaternion orient = msg->pose[base_id].orientation;
                    const geometry_msgs::msg::Vector3 lin = msg->twist[base_id].linear;
                    const geometry_msgs::msg::Vector3 ang = msg->twist[base_id].angular;

                    
                    this->q.head(7) << pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w;
                    this->v.head(6) << lin.x, lin.y, lin.z, ang.x, ang.y, ang.z;
                    
                }
            );   
        }
        else {
        // Use the state estimator measurements.

            estimated_pose_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
                "/state_estimator/pose", 1,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) -> void
                {
                    this->q.head(7) << msg->position.x, msg->position.y, msg->position.z,
                                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
                }
            );

            estimated_twist_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
                "/state_estimator/twist", 1,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
                {
                    this->v.head(6) << msg->linear.x, msg->linear.y, msg->linear.z,
                                msg->angular.x, msg->angular.y, msg->angular.z;
                }
            );
        }

        // #############################################################################################################################
        //std::cout <<"mpc step:\n" <<mpc.get_steps() <<"\n" <<std::endl;
        
        this->des_gen_pose_subscription = get_node()->create_subscription<generalized_pose_msgs::msg::GeneralizedPose>(
        "/motion_planner/desired_generalized_pose", 1,
        [this](const generalized_pose_msgs::msg::GeneralizedPose::SharedPtr msg) -> void
        {
            this->des_gen_poses_single.generalized_poses_with_time.resize(1);
            GeneralizedPoseWithTime gen_pose;
            gen_pose.gen_pose.base_acc << msg->base_acc.x, msg->base_acc.y, msg->base_acc.z;
            gen_pose.gen_pose.base_vel << msg->base_vel.x, msg->base_vel.y, msg->base_vel.z;
            gen_pose.gen_pose.base_pos << msg->base_pos.x, msg->base_pos.y, msg->base_pos.z;
            gen_pose.gen_pose.base_angvel << msg->base_angvel.x, msg->base_angvel.y, msg->base_angvel.z;
            gen_pose.gen_pose.base_quat << msg->base_quat.x, msg->base_quat.y, msg->base_quat.z, msg->base_quat.w;
            gen_pose.gen_pose.feet_acc = Eigen::VectorXd::Map(msg->feet_acc.data(), msg->feet_acc.size());
            gen_pose.gen_pose.feet_vel = Eigen::VectorXd::Map(msg->feet_vel.data(), msg->feet_vel.size());
            gen_pose.gen_pose.feet_pos = Eigen::VectorXd::Map(msg->feet_pos.data(), msg->feet_pos.size());
            gen_pose.gen_pose.contact_feet_names.assign(msg->contact_feet.data(), &msg->contact_feet[msg->contact_feet.size()]);

            gen_pose.time = 0;
            this->des_gen_poses_single.generalized_poses_with_time[0] = gen_pose;
        }
        );

        this->des_gen_poses_subscription = get_node()->create_subscription<generalized_pose_msgs::msg::GeneralizedPosesWithTime>(
        "/motion_planner/desired_generalized_poses", 1,
        [this](const generalized_pose_msgs::msg::GeneralizedPosesWithTime::SharedPtr msg) -> void
        {
            std::cout <<"msg size: " <<msg->generalized_poses_with_time.size() <<std::endl;
            if(msg->generalized_poses_with_time.size() <= this->mpc_step_horizon){
                mpc.set_steps(msg->generalized_poses_with_time.size());
            }
            else{
                mpc.set_steps(this->mpc_step_horizon);
            }
           // std::cout <<"mpc.get_steps() dentro subscriver: " <<mpc.get_steps() <<std::endl;

            this->des_gen_poses.generalized_poses_with_time.resize(mpc.get_steps());
            for (auto i=0; i<mpc.get_steps(); i++){

                //if(msg->generalized_poses_with_time[i].generalized_pose.feet_acc.empty()) std::cout <<"feet_acc empty\n" <<std::endl;
                GeneralizedPoseWithTime gen_pose;
                gen_pose.gen_pose.base_acc << msg->generalized_poses_with_time[i].generalized_pose.base_acc.x, msg->generalized_poses_with_time[i].generalized_pose.base_acc.y, msg->generalized_poses_with_time[i].generalized_pose.base_acc.z;
                gen_pose.gen_pose.base_vel << msg->generalized_poses_with_time[i].generalized_pose.base_vel.x, msg->generalized_poses_with_time[i].generalized_pose.base_vel.y, msg->generalized_poses_with_time[i].generalized_pose.base_vel.z;
                gen_pose.gen_pose.base_pos << msg->generalized_poses_with_time[i].generalized_pose.base_pos.x, msg->generalized_poses_with_time[i].generalized_pose.base_pos.y, msg->generalized_poses_with_time[i].generalized_pose.base_pos.z;
                gen_pose.gen_pose.base_angvel << msg->generalized_poses_with_time[i].generalized_pose.base_angvel.x, msg->generalized_poses_with_time[i].generalized_pose.base_angvel.y, msg->generalized_poses_with_time[i].generalized_pose.base_angvel.z;
                gen_pose.gen_pose.base_quat << msg->generalized_poses_with_time[i].generalized_pose.base_quat.x, msg->generalized_poses_with_time[i].generalized_pose.base_quat.y, msg->generalized_poses_with_time[i].generalized_pose.base_quat.z, msg->generalized_poses_with_time[i].generalized_pose.base_quat.w;
                gen_pose.gen_pose.feet_acc = Eigen::VectorXd::Map(msg->generalized_poses_with_time[i].generalized_pose.feet_acc.data(), msg->generalized_poses_with_time[i].generalized_pose.feet_acc.size());
                gen_pose.gen_pose.feet_vel = Eigen::VectorXd::Map(msg->generalized_poses_with_time[i].generalized_pose.feet_vel.data(), msg->generalized_poses_with_time[i].generalized_pose.feet_vel.size());
                gen_pose.gen_pose.feet_pos = Eigen::VectorXd::Map(msg->generalized_poses_with_time[i].generalized_pose.feet_pos.data(), msg->generalized_poses_with_time[i].generalized_pose.feet_pos.size());
                gen_pose.gen_pose.contact_feet_names.assign(msg->generalized_poses_with_time[i].generalized_pose.contact_feet.data(), &msg->generalized_poses_with_time[i].generalized_pose.contact_feet[msg->generalized_poses_with_time[i].generalized_pose.contact_feet.size()]);
                //gen_pose.gen_pose.contact_feet_names = {"LF", "RF", "LH", "RH"};

                //std::cout <<"contact_feet_names msg:\n" <<std::endl;
                //for(int k=0; k<gen_pose.gen_pose.contact_feet_names.size(); k++){
                  //  std::cout <<gen_pose.gen_pose.contact_feet_names[k] <<" " <<std::endl;
                //}

                gen_pose.time = 0;

                this->des_gen_poses.generalized_poses_with_time[i] = gen_pose;
            }
        }
        );
        // #############################################################################################################################

        if(logging){
            logger = std::make_shared<MPCPublisher>();
        }
    //std::cout <<"\n\n******************** Ending on configure ********************\n\n" <<std::endl;
    return CallbackReturn::SUCCESS;
    }

    /* =============================== On_activate ============================== */

    CallbackReturn MPCController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }


    /* ============================== On_deactivate ============================= */

    CallbackReturn MPCController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    /* ============================== Update ==============================*/

    controller_interface::return_type MPCController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/){
    
        //std::cout <<"\n\n******************* Spinning controller *******************\n\n";

        auto start = high_resolution_clock::now();
    
        std::vector<std::string> contact_feet_names = mpc.get_generic_feet_names();
        Eigen::VectorXd previous_state = Eigen::VectorXd::Zero(37);

        previous_state << this->q, this->v;

        //std::cout <<"state_interface size:\n" <<state_interfaces_.size() <<"\n" <<std::endl;
        for (uint i=0; i<this->joint_names.size(); i++) {
        	//std::cout <<"\n i: " <<i <<"\n" <<std::endl;
            this->q(i+7) = state_interfaces_[2*i].get_value();
            this->v(i+6) = state_interfaces_[2*i+1].get_value();
        }
        Eigen::VectorXd tau_controller = Eigen::VectorXd::Zero(joint_names.size() + 6);
        //std::cout <<"des_gen_poses_single size: " <<des_gen_poses_single.generalized_poses_with_time.size() <<std::endl;

        Eigen::VectorXd des_base_state = Eigen::VectorXd::Zero(6);
        //std::cout <<"des_gen_poses_single size " <<des_gen_poses_single.generalized_poses_with_time.size() <<std::endl;
        
        if((mpc.get_steps() == 1 && des_gen_poses_single.generalized_poses_with_time.empty()) || (mpc.get_steps()>1 && des_gen_poses.generalized_poses_with_time.empty())){      // The planner is not publishing messages yet. Interpolate from q0 to qi and than wait.
        //if(true){
        std::cout <<"\n\n******************* Entered if interpolate *******************\n\n" <<std::endl;
        Eigen::VectorXd q = std::min(1., time.seconds() / this->init_time) * this->q_init;

        // PD for the state estimator initialization
        for (uint i=0; i<this->joint_names.size(); i++) {
            command_interfaces_[i].set_value( Kp_PD*(q[i] - this->q(i+7)) + Kd_PD*(- this->v(i+6)) );
            tau_controller(i+6) = + 5*(q[i] - this->q(i+7)) + 0.05*(- this->v(i+6));
            //std::cout <<"command interface[" <<i <<"]:\n" <<command_interfaces_[i].get_value() <<"\n" <<std::endl;
        }
        mpc.set_tau(tau_controller);
        
    } else {
        
        //std::cout <<"\n\n******************* Entered else *******************\n\n" <<std::endl;
        if(mpc_step_horizon == 1){
            //std::cout <<"des_gen_poses.size: " <<des_gen_poses_single.generalized_poses_with_time.size() <<std::endl;
            //std::cout <<"q dentro mpc_controller:\n" <<q <<std::endl;
            //std::cout <<"v dentro mpc_controller:\n" <<v <<std::endl;
            //std::cout <<"des base acc\n" <<des_gen_poses_single.generalized_poses_with_time[0].gen_pose.base_acc <<std::endl;
            //std::cout <<"des base vel\n" <<des_gen_poses_single.generalized_poses_with_time[0].gen_pose.base_vel <<std::endl;
            //std::cout <<"des base pos\n" <<des_gen_poses_single.generalized_poses_with_time[0].gen_pose.base_pos <<std::endl;
            //for (int p=0; p<des_gen_poses_single.generalized_poses_with_time[0].gen_pose.contact_feet_names.size(); p++){
                //std::cout <<"contact name [p]: " <<des_gen_poses_single.generalized_poses_with_time[0].gen_pose.contact_feet_names[p] <<std::endl;
            //}
            this->tau = mpc.solve_MPC(this->q, this->v, this->des_gen_poses_single);
            des_base_state.head(3) = des_gen_poses_single.generalized_poses_with_time[0].gen_pose.base_pos;
            des_base_state.tail(3) = des_gen_poses_single.generalized_poses_with_time[0].gen_pose.base_vel;
            //this->pid_gains = mpc.tune_gains(this->q, this->v, this->des_gen_poses);
        }
        else{
            //std::cout <<"des poses size prima di chiamata a solve: " <<des_gen_poses.generalized_poses_with_time.size() <<std::endl;
            //std::cout <<"mpc.get_steps() prima di chiamata a solve: " <<mpc.get_steps() <<std::endl;
            this->tau = mpc.solve_MPC(this->q, this->v, this->des_gen_poses);
            //des_base_state.head(3) = des_gen_poses.generalized_poses_with_time[0].gen_pose.base_pos;
            //des_base_state.tail(3) = des_gen_poses.generalized_poses_with_time[0].gen_pose.base_vel;
            //this->pid_gains = mpc.tune_gains(this->q, this->v, this->des_gen_poses);
        }

        // Send effort command
        for (uint i=0; i<joint_names.size(); i++) {
            command_interfaces_[i].set_value(tau[0](i));
        }
        Eigen::VectorXd q_opt = mpc.get_robot().get_optimal_q()[0];
        Eigen::VectorXd v_opt = mpc.get_robot().get_optimal_v()[0];

        
        if(logging)
            logger->publish_all(
                tau[0], pid_gains, q_opt, v_opt, des_base_state, mpc.get_feet_positions(), mpc.get_feet_velocities(this->v)
            );
        }

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop-start);
        std::cout <<"Durata in millisecondi: " <<duration.count() <<std::endl;

        return controller_interface::return_type::OK;
    }
}


PLUGINLIB_EXPORT_CLASS(
    mpc_controller::MPCController,
    controller_interface::ControllerInterface
)
