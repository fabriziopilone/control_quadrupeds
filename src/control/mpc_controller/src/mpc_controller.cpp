#include "mpc_controller/mpc_controller.hpp"
#include "mpc_controller/mpc_publisher.hpp"

namespace mpc_controller{

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

    MPCController::MPCController():mpc("solo12", 1.0/400){}
    
    CallbackReturn MPCController::on_init(){
        try{
            auto_declare<std::string>("robot_name", std::string());
            auto_declare<double>("sample_time", double());
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>);
            auto_declare<bool>("use_estimator", bool());

            auto_declare<std::vector<double>>("q_init", std::vector<double>());

            auto_declare<double>("tau_max", double());
            auto_declare<double>("tau_min", double());
            auto_declare<double>("f_max", double());
            auto_declare<double>("f_min", double());
            auto_declare<souble>("mu", double());

            // auto_declare<double>("reg", double());

        }
        catch(const std::exception& e) {
        fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    }

    CallbackReturn MPCController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
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
        if (joint_names.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "joints is empty");
            return CallbackReturn::ERROR;
        }

        const bool use_estimator = get_node()->get_parameter("use_estimator").as_bool();

        auto q_i = get_node()->get_parameter("q_init").as_double_array();
        if(q_i.size() != 12){
            RCLCPP_ERROR(get_node()->get_logger(), "q_init does not have 12 elements");
            return CallbackReturn::ERROR;
        }
        this->q_init = Eigen::VectorXd::Map(q_i.data(), q_i.size());



        mpc = MPC(robot_name, dT);

        this->q.resize(mpc.get_robot().get_state_dim() + 1);
        q(6) = 1;   // ????

        this->des_gen_pose_.feet_pos.resize(3);
        this->des_gen_pose_.feet_vel.resize(3);
        this->des_gen_pose_.feet_acc.resize(3);         // XXXXXXXXXXXXXXXXXXXXXXXXXX
        this->des_gen_pose_.feet_pos.resize(0);
        this->des_gen_pose_.feet_vel.resize(0);
        this->des_gen_pose_.feet_acc.resize(0);


        if (get_node()->get_parameter("tau_max").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'tau_max' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.get_robot().set_tau_max(get_node()->get_parameter("tau_max").as_double());

        if (get_node()->get_parameter("tau_min").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'tau_min' parameter is < 0");
            return CallbackReturn::ERROR;
        }
        mpc.get_robot().set_tau_min(get_node()->get_parameter("tau_min").as_double());
    
        if (get_node()->get_parameter("mu").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'mu' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.set_mu(get_node()->get_parameter("mu").as_double());
        
        if (get_node()->get_parameter("f_max").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'f_max' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.get_robot().set_f_max(get_node()->get_parameter("f_max").as_double());
        
        if (get_node()->get_parameter("f_min").as_double() <= 0) {
            RCLCPP_ERROR(get_node()->get_logger(),"'f_min' parameter is <= 0");
            return CallbackReturn::ERROR;
        }
        mpc.get_robot().set_f_min(get_node()->get_parameter("f_min").as_double());



        if (!use_estimator){
            // Use the simulated data of Gazebo

            this->joint_state_subscription = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
                "/gazebo/link_states", 1,
                [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg)->void{
                    int base_id = -1;

                    for(std::size_t i00; i<msg->name.size(); i++){
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

                    this->q(7) << pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w;
                    this->q_dot(6) << lin.x, lin.y. lin.z, ang.x, ang.y, ang.z;
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
    
        desired_generalized_pose_subscription_ = get_node()->create_subscription<generalized_pose_msgs::msg::GeneralizedPosesWithTime>(
        "/motion_planner/desired_generalized_poses", mpc.get_steps(),
        [this](const generalized_pose_msgs::msg::GeneralizedPose::SharedPtr msg) -> void
        {
            for (int i=0; i<mpc.get_steps(); i++){
                this->des_gen_poses[i].base_acc << msg->base_acc.x, msg->base_acc.y, msg->base_acc.z;
                this->des_gen_pose[i].base_vel << msg->base_vel.x, msg->base_vel.y, msg->base_vel.z;
                this->des_gen_pose[i].base_pos << msg->base_pos.x, msg->base_pos.y, msg->base_pos.z;

                this->des_gen_pose[i].base_angvel << msg->base_angvel.x, msg->base_angvel.y, msg->base_angvel.z;
                this->des_gen_pose[i].base_quat << msg->base_quat.x, msg->base_quat.y, msg->base_quat.z, msg->base_quat.w;

                this->des_gen_pose[i].feet_acc = Eigen::VectorXd::Map(msg->feet_acc.data(), msg->feet_acc.size());
                this->des_gen_pose[i].feet_vel = Eigen::VectorXd::Map(msg->feet_vel.data(), msg->feet_vel.size());
                this->des_gen_pose[i].feet_pos = Eigen::VectorXd::Map(msg->feet_pos.data(), msg->feet_pos.size());

                this->des_gen_pose[i].contact_feet_names.assign(msg->contact_feet.data(), &msg->contact_feet[msg->contact_feet.size()]);
            }
        }
        );

    return CallbackReturn::SUCCESS;
    }

    /* ===================== Command_interface_configuration ==================== */

    InterfaceConfiguration MPCController::command_interface_configuration() const
    {
        InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : joint_names_) {
            command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
        }

        return command_interfaces_config;
    }


    /* ====================== State_interface_configuration ===================== */

    InterfaceConfiguration MPCController::state_interface_configuration() const
    {
        InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : joint_names_) {
            state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
            state_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
        }
        
        return state_interfaces_config;
    }

    