#include "single_arm_hardware_interface/single_arm_hardware_interface.hpp"
#include "device_interface/DeviceDriver/device_interface.h"
#include "device_interface/Base/RobotStruct.h"
#include "device_interface/Base/RobotSystem.h"
#include "device_interface/Move/MovePlan.h"
#include "device_interface/Teach/robot_teach_interface.h"
namespace single_arm_hardware_interface
{

SingleArmHardwareInterface::SingleArmHardwareInterface() :
_sim_flag(false), 
mode(e_mode::in_Position),
logger_(rclcpp::get_logger("resource_manager"))
// node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("HardwareDriver"))
{}

SingleArmHardwareInterface::~SingleArmHardwareInterface(){}

e_mode SingleArmHardwareInterface::convertToEMode(signed char value) {
    switch (value) {
        case 8: return e_mode::in_Position;
        case 9: return e_mode::in_Velocity;
        case 10: return e_mode::in_Effort;
        default: return e_mode::in_Position;
    }
}

hardware_interface::CallbackReturn SingleArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){

    /*
        Get hardware info from urdf.
    */
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }

    /*
        get parameter system_arg, initialize system.
    */
    std::string system_arg;
    auto it = info_.hardware_parameters.find("system_arg");
    if (it == info_.hardware_parameters.end()){
        RCLCPP_ERROR(logger_, "Ros2 can't find parammeter \"system_arg\"");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        system_arg = it->second;
        RCLCPP_INFO(logger_, "parammeter \"system_arg\" = %s", system_arg.c_str());
    }

    HYYRobotBase::command_arg arg;
    if (0 == HYYRobotBase::commandLineParser1(system_arg.c_str(), &arg)){
        if (0 != HYYRobotBase::system_initialize(&arg)){
            RCLCPP_ERROR(logger_, "device initialize failure! system_initialize faiure!");
        }
    }else{
        RCLCPP_ERROR(logger_, "device initialize failure! commandLineParser1 faiure!");
    } 

    /*
        Get component(robots and addGroups) name and dof.
    */
    robot.component_name = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), 0);
    robot._index = HYYRobotBase::get_robot_index();
    robot._dof = HYYRobotBase::get_group_dof(robot.component_name);
    RCLCPP_INFO(logger_, "robot name:  %s.", robot.component_name);
    RCLCPP_INFO(logger_, "robot index: %d.", robot._index );
    RCLCPP_INFO(logger_, "robot dof:   %zu.", robot._dof);

    /*
        Get each component joints.
    */
    for (const hardware_interface::ComponentInfo & joint : info_.joints){ 
        robot.joints.push_back(joint.name);
    }

    /*
        Check each component joints number.
    */
    robot.joints_num = robot.joints.size();
    if (robot._dof != robot.joints_num){
        RCLCPP_ERROR(logger_, "robot device dof(%zu) != joint num(%zu).", robot._dof, robot.joints_num);
        return hardware_interface::CallbackReturn::ERROR;
    }

    /*
        Get parameter "sim_flag"
    */
    it = info_.hardware_parameters.find("sim_flag");
    if (it == info_.hardware_parameters.end()){
        RCLCPP_ERROR(logger_, "can't find parammeter \"sim_flag\"");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        std::string tem = "True";
        _sim_flag = (tem == it->second);
        RCLCPP_INFO(logger_, "parammeter \"sim_flag\" = %s.", _sim_flag ? "True" : "False");
    }

    /*
        Get and Set parameter "device_mode"
    */
    int e_m = 0;
    it = info_.hardware_parameters.find("device_mode");
    if (!_sim_flag){
        if (it == info_.hardware_parameters.end()){
            RCLCPP_ERROR(logger_, "can't find parammeter \"device_mode\"");
            return hardware_interface::CallbackReturn::ERROR;
        }else{
            e_m = std::stoi(it->second);
            RCLCPP_INFO(logger_, "parammeter \"device_mode\" = %d", e_m);
            if (e_m == 8 || e_m == 9 || e_m == 10){
                robot.device_mode = (signed char)e_m;
                HYYRobotBase::set_group_mode(robot.component_name, &(robot.device_mode));
                RCLCPP_INFO(logger_, "robot run mode is set to %d", robot.device_mode);
            }else{
                RCLCPP_ERROR(logger_, "parammeter \"device_mode\" value error.");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    /*
        Initialize robots raw data
    */
    try{

        robot.joint_position_state_.resize(robot.joints_num);
        robot.joint_velocity_state_.resize(robot.joints_num);
        robot.joint_effort_state_.resize(robot.joints_num);
        robot.joint_position_command_.resize(robot.joints_num);
        robot.joint_velocity_command_.resize(robot.joints_num);
        robot.joint_effort_command_.resize(robot.joints_num);

        for (size_t j = 0; j < robot.joints_num; j++){
            robot.joint_position_state_[j] = HYYRobotBase::GetAxisPosition(robot.component_name, j + 1);
            robot.joint_velocity_state_[j] = 0.0;
            robot.joint_effort_state_[j] = 0.0;
            robot.joint_position_command_[j] = robot.joint_position_state_[j];
            robot.joint_velocity_command_[j] = robot.joint_velocity_state_[j];
            robot.joint_effort_command_[j] = robot.joint_effort_state_[j];
        }
        
    }catch(std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> SingleArmHardwareInterface::export_state_interfaces(){

    RCLCPP_INFO(logger_, "exporting state interfaces");
    std::vector<hardware_interface::StateInterface> result;
    try{        
        for (size_t j = 0; j < robot.joints_num; j++){
            result.emplace_back(hardware_interface::StateInterface(
                robot.joints[j], hardware_interface::HW_IF_POSITION, &robot.joint_position_state_[j]));
            result.emplace_back(hardware_interface::StateInterface(
                robot.joints[j], hardware_interface::HW_IF_VELOCITY, &robot.joint_velocity_state_[j]));
            result.emplace_back(hardware_interface::StateInterface(
                robot.joints[j], hardware_interface::HW_IF_EFFORT, &robot.joint_effort_state_[j]));
        } 
    }catch (std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
    }

    return result;
}

std::vector<hardware_interface::CommandInterface> SingleArmHardwareInterface::export_command_interfaces(){

    RCLCPP_INFO(logger_, "exporting command interfaces");
    std::vector<hardware_interface::CommandInterface> result;
    try{
        for (size_t i = 0; i < robot.joints_num; i++){
            result.emplace_back(hardware_interface::CommandInterface(
                robot.joints[i], hardware_interface::HW_IF_POSITION, &robot.joint_position_command_[i]));
            result.emplace_back(hardware_interface::CommandInterface(
                robot.joints[i], hardware_interface::HW_IF_VELOCITY, &robot.joint_velocity_command_[i]));
            result.emplace_back(hardware_interface::CommandInterface(
                robot.joints[i], hardware_interface::HW_IF_EFFORT, &robot.joint_effort_command_[i]));
        }
    }catch (std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
    }
    
    return result;
}

hardware_interface::CallbackReturn SingleArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){    

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SingleArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){

    HYYRobotBase::RobotPoweroff(robot._index);

    HYYRobotBase::set_robot_run_type(_sim_flag);

    HYYRobotBase::RobotPower(robot._index);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SingleArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    
    // HYYRobotBase::set_robot_run_type(0);
    
    HYYRobotBase::RobotPoweroff(robot._index);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SingleArmHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    mode = convertToEMode(robot.device_mode);
    if (mode == e_mode::in_Position){
        HYYRobotBase::GetGroupPosition(robot.component_name, &(robot.joint_position_state_[0]));
        // RCLCPP_INFO(logger_, "read  robot joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_position_state_[0],
        //             robot.joint_position_state_[1], robot.joint_position_state_[2], robot.joint_position_state_[3],
        //             robot.joint_position_state_[4], robot.joint_position_state_[5], robot.joint_position_state_[6]);
    }else if(mode  == e_mode::in_Velocity){
        HYYRobotBase::GetGroupVelocity(robot.component_name, &(robot.joint_velocity_state_[0]));
        // RCLCPP_INFO(logger_, "read  robot joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_velocity_state_[0],
        //             robot.joint_velocity_state_[1], robot.joint_velocity_state_[2], robot.joint_velocity_state_[3],
        //             robot.joint_velocity_state_[4], robot.joint_velocity_state_[5], robot.joint_velocity_state_[6]);
    }else if (mode == e_mode::in_Effort){
        HYYRobotBase::GetGroupTorque(robot.component_name, &(robot.joint_effort_state_[0]));
        // RCLCPP_INFO(logger_, "read  robot joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_effort_state_[0],
        //             robot.joint_effort_state_[1], robot.joint_effort_state_[2], robot.joint_effort_state_[3],
        //             robot.joint_effort_state_[4], robot.joint_effort_state_[5], robot.joint_effort_state_[6]);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SingleArmHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    
    mode = convertToEMode(robot.device_mode);
    if (mode == e_mode::in_Position){
        HYYRobotBase::SetGroupPosition(robot.component_name, &(robot.joint_position_command_[0]));
        // RCLCPP_INFO(logger_, "write robot joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_position_command_[0],
        //             robot.joint_position_command_[1], robot.joint_position_command_[2], robot.joint_position_command_[3],
        //             robot.joint_position_command_[4], robot.joint_position_command_[5], robot.joint_position_command_[6]);
    }else if (mode == e_mode::in_Velocity){
        HYYRobotBase::SetGroupVelocity(robot.component_name, &(robot.joint_velocity_command_[0]));
        // RCLCPP_INFO(logger_, "write robot joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_velocity_command_[0],
        //             robot.joint_velocity_command_[1], robot.joint_velocity_command_[2], robot.joint_velocity_command_[3],
        //             robot.joint_velocity_command_[4], robot.joint_velocity_command_[5], robot.joint_velocity_command_[6]);
    }else if (mode == e_mode::in_Effort){
        HYYRobotBase::SetGroupTorque(robot.component_name, &(robot.joint_effort_command_[0]));
        // RCLCPP_INFO(logger_, "write robot joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", robot.joint_effort_command_[0],
        //             robot.joint_effort_command_[1], robot.joint_effort_command_[2], robot.joint_effort_command_[3],
        //             robot.joint_effort_command_[4], robot.joint_effort_command_[5], robot.joint_effort_command_[6]);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace single_arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(single_arm_hardware_interface::SingleArmHardwareInterface, hardware_interface::SystemInterface)
