#include "hyy_hardware_interface/hyy_hardware_interface.hpp"
#include "device_interface/DeviceDriver/device_interface.h"
#include "device_interface/Base/RobotStruct.h"
#include "device_interface/Base/RobotSystem.h"
#include "device_interface/Move/MovePlan.h"
#include "device_interface/Teach/robot_teach_interface.h"

namespace hyy_hardware_interface
{

HyyHardwareInterface::HyyHardwareInterface() :
robots_num(0),
addGroups_num(0),
_sim_flag(false), 
if_add_axisgroups(true), 
mode(e_mode::in_Position),
logger_(rclcpp::get_logger("resource_manager"))
{}

HyyHardwareInterface::~HyyHardwareInterface(){}

e_mode HyyHardwareInterface::convertToEMode(signed char value) {
    switch (value) {
        case 8: return e_mode::in_Position;
        case 9: return e_mode::in_Velocity;
        case 10: return e_mode::in_Effort;
        default: return e_mode::in_Position;
    }
}

hardware_interface::CallbackReturn HyyHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){

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
        get parameter if_add_axisgroups & if_add_external_device
    */
    it = info_.hardware_parameters.find("if_add_axisgroups");
    if (it == info_.hardware_parameters.end()){
        RCLCPP_ERROR(logger_, "Ros2 can't find parammeter \"if_add_axisgroups\"");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        std::string tem = "True";
        if_add_axisgroups = (tem == it->second);
        RCLCPP_INFO(logger_, "parammeter \"if_add_axisgroups\" = %s.", if_add_axisgroups ? "True" : "False");
    }

    it = info_.hardware_parameters.find("if_add_external_device");
    if (it == info_.hardware_parameters.end()){
        RCLCPP_ERROR(logger_, "Ros2 can't find parammeter \"if_add_external_device\"");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        std::string tem = "True";
        if_add_external_device = (tem == it->second);
        RCLCPP_INFO(logger_, "parammeter \"if_add_external_device\" = %s.", if_add_external_device ? "True" : "False");
    }

    /*
        Get component(robots and addGroups) name and dof.
    */
    robots_num = HYYRobotBase::hasNumber_robot_device(HYYRobotBase::get_deviceName(0, NULL));
    robots.resize(robots_num);
    for (int i = 0; i < robots_num; i++)
    {
        robots[i].component_name = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), i);
        robots[i]._dof = HYYRobotBase::get_group_dof(robots[i].component_name);
        robots[i]._index = HYYRobotBase::get_index_robot_device(robots[i].component_name);
        RCLCPP_INFO(logger_, "robot(%d) name: %s, dof: %ld.", i, robots[i].component_name, robots[i]._dof);
    }
    if (if_add_axisgroups){
        addGroups_num = HYYRobotBase::hasNumber_additionaxis_device(HYYRobotBase::get_deviceName(0, NULL));
        addGroups.resize(addGroups_num);
        for (int i = 0; i < addGroups_num; i++)
        {
            addGroups[i].component_name = HYYRobotBase::get_name_additionaxis_device(HYYRobotBase::get_deviceName(0, NULL), i);
            addGroups[i]._dof = HYYRobotBase::get_group_dof(addGroups[i].component_name);
            addGroups[i]._index = HYYRobotBase::get_index_additionaxis_device(addGroups[i].component_name);
            RCLCPP_INFO(logger_, "addGroups(%d) name: %s, dof: %ld.", i, addGroups[i].component_name, addGroups[i]._dof);
        }
    }
    if (if_add_external_device){
        it = info_.hardware_parameters.find("external_device_dof");
        if (it == info_.hardware_parameters.end()){
            RCLCPP_ERROR(logger_, "Ros2 can't find parammeter \"external_device_dof\"");
            return hardware_interface::CallbackReturn::ERROR;
        }else{
            externalDevices._dof = std::stoi(it->second);
            RCLCPP_INFO_STREAM(logger_, "parammeter \"external_device_dof\" = " << externalDevices._dof);
        }
    }

    /*
        Get each component joints.
        robot(0,1,2...) + addGroup(0,1,2...)
    */
    for (const hardware_interface::ComponentInfo & joint : info_.joints){ 
        joints.push_back(joint.name);
    }
    int index = 0; 
    for (size_t i = 0; i < robots.size(); ++i) {
        for (size_t j = 0; j < robots[i]._dof; ++j) {
            robots[i].joints.push_back(joints[index++]);
        }
    }
    if (if_add_axisgroups){
        for (size_t i = 0; i < addGroups.size(); ++i){
            for (size_t j = 0; j < addGroups[i]._dof; ++j){
                addGroups[i].joints.push_back(joints[index++]);
            }
        }
    }
    if (if_add_external_device){ 
        for (size_t i = 0; i < externalDevices._dof; ++i){
            externalDevices.joints.push_back(joints[index++]);
        }
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
                for (int i = 0; i < robots_num; i++){
                    robots[i].device_mode = (signed char)e_m;
                    HYYRobotBase::set_group_mode(robots[i].component_name, &(robots[i].device_mode));
                    RCLCPP_INFO(logger_, "robot(%d) run mode is set to %d", i, robots[i].device_mode);
                }
                if (if_add_axisgroups){
                    for (int i = 0; i < addGroups_num; i++){
                        addGroups[i].device_mode = (signed char)e_m;
                        HYYRobotBase::set_group_mode(addGroups[i].component_name, &(addGroups[i].device_mode));
                        RCLCPP_INFO(logger_, "addGroups(%d) run mode is set to %d", i, addGroups[i].device_mode);
                    }
                }
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
        for (int i = 0; i < robots_num; i++)
        {
            robots[i].joint_position_state_.resize(robots[i]._dof);
            robots[i].joint_velocity_state_.resize(robots[i]._dof);
            robots[i].joint_effort_state_.resize(robots[i]._dof);
            robots[i].joint_position_command_.resize(robots[i]._dof);
            robots[i].joint_velocity_command_.resize(robots[i]._dof);
            robots[i].joint_effort_command_.resize(robots[i]._dof);

            for (size_t j = 0; j < robots[i]._dof; j++){
                robots[i].joint_position_state_[j] = HYYRobotBase::GetAxisPosition(robots[i].component_name, j + 1);
                robots[i].joint_velocity_state_[j] = 0.0;
                robots[i].joint_effort_state_[j] = 0.0;
                robots[i].joint_position_command_[j] = robots[i].joint_position_state_[j];
                robots[i].joint_velocity_command_[j] = robots[i].joint_velocity_state_[j];
                robots[i].joint_effort_command_[j] = robots[i].joint_effort_state_[j];
            }
        }

        if (if_add_axisgroups){
            for (int i = 0; i < addGroups_num; i++)
            {
                addGroups[i].joint_position_state_.resize(addGroups[i]._dof);
                addGroups[i].joint_velocity_state_.resize(addGroups[i]._dof);
                addGroups[i].joint_effort_state_.resize(addGroups[i]._dof);
                addGroups[i].joint_position_command_.resize(addGroups[i]._dof);
                addGroups[i].joint_velocity_command_.resize(addGroups[i]._dof);
                addGroups[i].joint_effort_command_.resize(addGroups[i]._dof);

                for (size_t j = 0; j < addGroups[i]._dof; j++){
                    addGroups[i].joint_position_state_[j] = HYYRobotBase::GetAxisPosition(addGroups[i].component_name, j + 1);
                    addGroups[i].joint_velocity_state_[j] = 0.0;
                    addGroups[i].joint_effort_state_[j] = 0.0;
                    addGroups[i].joint_position_command_[j] = addGroups[i].joint_position_state_[j];
                    addGroups[i].joint_velocity_command_[j] = addGroups[i].joint_velocity_state_[j];
                    addGroups[i].joint_effort_command_[j] = addGroups[i].joint_effort_state_[j];
                }
            }   
        }

        if (if_add_external_device){
            externalDevices.joint_position_state_.resize(externalDevices._dof);
            externalDevices.joint_position_command_.resize(externalDevices._dof);
            for (size_t i = 0; i < externalDevices._dof; i++){
                externalDevices.joint_position_state_[i] = 0.0;
                externalDevices.joint_position_command_[i] = externalDevices.joint_position_state_[i];
            }
        }
        
    }catch(std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HyyHardwareInterface::export_state_interfaces(){

    RCLCPP_INFO(logger_, "exporting state interfaces");
    std::vector<hardware_interface::StateInterface> result;
    try{     
        for (int i = 0; i < robots_num; i++){  
            for (size_t j = 0; j < robots[i]._dof; j++){
                result.emplace_back(hardware_interface::StateInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_POSITION, &robots[i].joint_position_state_[j]));
                result.emplace_back(hardware_interface::StateInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_VELOCITY, &robots[i].joint_velocity_state_[j]));
                result.emplace_back(hardware_interface::StateInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_EFFORT, &robots[i].joint_effort_state_[j]));
            } 
        }
        if (if_add_axisgroups){
            for (int i = 0; i < addGroups_num; i++){  
                for (size_t j = 0; j < addGroups[i]._dof; j++){
                    result.emplace_back(hardware_interface::StateInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_POSITION, &addGroups[i].joint_position_state_[j]));
                    result.emplace_back(hardware_interface::StateInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_VELOCITY, &addGroups[i].joint_velocity_state_[j]));
                    result.emplace_back(hardware_interface::StateInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_EFFORT, &addGroups[i].joint_effort_state_[j]));
                } 
            }
        }
        if (if_add_external_device){
                for (size_t i = 0; i < externalDevices._dof; i++){
                    result.emplace_back(hardware_interface::StateInterface(
                        externalDevices.joints[i], hardware_interface::HW_IF_POSITION, &externalDevices.joint_position_state_[i]));
                } 
        }
        
    }catch (std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
    }

    return result;
}

std::vector<hardware_interface::CommandInterface> HyyHardwareInterface::export_command_interfaces(){

    RCLCPP_INFO(logger_, "exporting command interfaces");
    std::vector<hardware_interface::CommandInterface> result;
    try{     
        for (int i = 0; i < robots_num; i++){  
            for (size_t j = 0; j < robots[i]._dof; j++){
                result.emplace_back(hardware_interface::CommandInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_POSITION, &robots[i].joint_position_command_[j]));
                result.emplace_back(hardware_interface::CommandInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_VELOCITY, &robots[i].joint_velocity_command_[j]));
                result.emplace_back(hardware_interface::CommandInterface(
                    robots[i].joints[j], hardware_interface::HW_IF_EFFORT, &robots[i].joint_effort_command_[j]));
            } 
        }
        if (if_add_axisgroups){
            for (int i = 0; i < addGroups_num; i++){  
                for (size_t j = 0; j < addGroups[i]._dof; j++){
                    result.emplace_back(hardware_interface::CommandInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_POSITION, &addGroups[i].joint_position_command_[j]));
                    result.emplace_back(hardware_interface::CommandInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_VELOCITY, &addGroups[i].joint_velocity_command_[j]));
                    result.emplace_back(hardware_interface::CommandInterface(
                        addGroups[i].joints[j], hardware_interface::HW_IF_EFFORT, &addGroups[i].joint_effort_command_[j]));
                } 
            }
        }
        if (if_add_external_device){
                for (size_t i = 0; i < externalDevices._dof; i++){
                    result.emplace_back(hardware_interface::CommandInterface(
                        externalDevices.joints[i], hardware_interface::HW_IF_POSITION, &externalDevices.joint_position_command_[i]));
                } 
        }   
    }catch (std::exception &e){
        RCLCPP_FATAL(logger_, "Error: %s", e.what());
    }

    return result;
}

hardware_interface::CallbackReturn HyyHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){    

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HyyHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){

    for (int i = 0; i < robots_num; i++){
        HYYRobotBase::RobotPoweroff(robots[i]._index);
    }
    
    for (int i = 0; i < addGroups_num; i++){
        HYYRobotBase::AdditionPoweroff(addGroups[i]._index);
    }

    HYYRobotBase::set_robot_run_type(_sim_flag);
    int runmode = HYYRobotBase::get_robot_run_type();
    RCLCPP_INFO(logger_, "robot current run type: %s(%d)", runmode ? "inSim" : "inReal", runmode);

    for (int i = 0; i < robots_num; i++){
        HYYRobotBase::RobotPower(robots[i]._index);
    }
    
    for (int i = 0; i < addGroups_num; i++){
        HYYRobotBase::AdditionPower(addGroups[i]._index);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HyyHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    
    // HYYRobotBase::set_robot_run_type(0);
    
    for (int i = 0; i < robots_num; i++){
        HYYRobotBase::RobotPoweroff(robots[i]._index);
    }
    
    for (int i = 0; i < addGroups_num; i++){
        HYYRobotBase::group_power_off(addGroups[i].component_name);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HyyHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    for (int i = 0; i < robots_num; i++){
        mode = convertToEMode(robots[i].device_mode);
        if (mode == e_mode::in_Position){
            HYYRobotBase::GetGroupPosition(robots[i].component_name, &(robots[i].joint_position_state_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_position_state_[0],
            //             robots[i].joint_position_state_[1], robots[i].joint_position_state_[2], robots[i].joint_position_state_[3],
            //             robots[i].joint_position_state_[4], robots[i].joint_position_state_[5], robots[i].joint_position_state_[6]);
        }else if(mode  == e_mode::in_Velocity){
            HYYRobotBase::GetGroupVelocity(robots[i].component_name, &(robots[i].joint_velocity_state_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_velocity_state_[0],
            //             robots[i].joint_velocity_state_[1], robots[i].joint_velocity_state_[2], robots[i].joint_velocity_state_[3],
            //             robots[i].joint_velocity_state_[4], robots[i].joint_velocity_state_[5], robots[i].joint_velocity_state_[6]);
        }else if (mode == e_mode::in_Effort){
            HYYRobotBase::GetGroupTorque(robots[i].component_name, &(robots[i].joint_effort_state_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_effort_state_[0],
            //             robots[i].joint_effort_state_[1], robots[i].joint_effort_state_[2], robots[i].joint_effort_state_[3],
            //             robots[i].joint_effort_state_[4], robots[i].joint_effort_state_[5], robots[i].joint_effort_state_[6]);
        }
    }
    if (if_add_axisgroups){
        for (int i = 0; i < addGroups_num; i++){
            mode = convertToEMode(addGroups[i].device_mode);
            if (mode == e_mode::in_Position){
                HYYRobotBase::GetGroupPosition(addGroups[i].component_name, &(addGroups[i].joint_position_state_[0]));
                // RCLCPP_INFO(logger_, "read addGroups(%d) joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addGroups[i].joint_position_state_[0],
                //             addGroups[i].joint_position_state_[1], addGroups[i].joint_position_state_[2], addGroups[i].joint_position_state_[3],
                //             addGroups[i].joint_position_state_[4], addGroups[i].joint_position_state_[5], addGroups[i].joint_position_state_[6]);
            }else if(mode  == e_mode::in_Velocity){
                HYYRobotBase::GetGroupVelocity(addGroups[i].component_name, &(addGroups[i].joint_velocity_state_[0]));
                // RCLCPP_INFO(logger_, "read addGroups(%d) joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addGroups[i].joint_velocity_state_[0],
                //             addGroups[i].joint_velocity_state_[1], addGroups[i].joint_velocity_state_[2], addGroups[i].joint_velocity_state_[3],
                //             addGroups[i].joint_velocity_state_[4], addGroups[i].joint_velocity_state_[5], addGroups[i].joint_velocity_state_[6]);
            }else if (mode == e_mode::in_Effort){
                HYYRobotBase::GetGroupTorque(addGroups[i].component_name, &(addGroups[i].joint_effort_state_[0]));
                // RCLCPP_INFO(logger_, "read addGroups(%d) joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addGroups[i].joint_effort_state_[0],
                //             addGroups[i].joint_effort_state_[1], addGroups[i].joint_effort_state_[2], addGroups[i].joint_effort_state_[3],
                //             addGroups[i].joint_effort_state_[4], addGroups[i].joint_effort_state_[5], addGroups[i].joint_effort_state_[6]);
            }
        }
    }
    if(if_add_external_device){
        // RCLCPP_INFO(logger_, "read externalDevice joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", externalDevices.joint_position_state_[0],
        //     externalDevices.joint_position_state_[1], externalDevices.joint_position_state_[2], externalDevices.joint_position_state_[3],
        //     externalDevices.joint_position_state_[4], externalDevices.joint_position_state_[5], externalDevices.joint_position_state_[6]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HyyHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    
    for (int i = 0; i < robots_num; i++){
        mode = convertToEMode(robots[i].device_mode);
        if (mode == e_mode::in_Position){
            HYYRobotBase::SetGroupPosition(robots[i].component_name, &(robots[i].joint_position_command_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_position_command_[0],
            //             robots[i].joint_position_command_[1], robots[i].joint_position_command_[2], robots[i].joint_position_command_[3],
            //             robots[i].joint_position_command_[4], robots[i].joint_position_command_[5], robots[i].joint_position_command_[6]);
        }else if(mode  == e_mode::in_Velocity){
            HYYRobotBase::SetGroupVelocity(robots[i].component_name, &(robots[i].joint_velocity_command_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_velocity_command_[0],
            //             robots[i].joint_velocity_command_[1], robots[i].joint_velocity_command_[2], robots[i].joint_velocity_command_[3],
            //             robots[i].joint_velocity_command_[4], robots[i].joint_velocity_command_[5], robots[i].joint_velocity_command_[6]);
        }else if (mode == e_mode::in_Effort){
            HYYRobotBase::SetGroupTorque(robots[i].component_name, &(robots[i].joint_effort_command_[0]));
            // RCLCPP_INFO(logger_, "read robot(%d) joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, robots[i].joint_effort_command_[0],
            //             robots[i].joint_effort_command_[1], robots[i].joint_effort_command_[2], robots[i].joint_effort_command_[3],
            //             robots[i].joint_effort_command_[4], robots[i].joint_effort_command_[5], robots[i].joint_effort_command_[6]);
        }
    }
    if (if_add_axisgroups){
        for (int i = 0; i < addGroups_num; i++){
            mode = convertToEMode(addGroups[i].device_mode);
            if (mode == e_mode::in_Position){
                HYYRobotBase::SetGroupPosition(addGroups[i].component_name, &(addGroups[i].joint_position_command_[0]));
                // RCLCPP_INFO(logger_, "read addgroup(%d) joints position: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addgroups[i].joint_position_command_[0],
                //             addgroups[i].joint_position_command_[1], addgroups[i].joint_position_command_[2], addgroups[i].joint_position_command_[3],
                //             addgroups[i].joint_position_command_[4], addgroups[i].joint_position_command_[5], addgroups[i].joint_position_command_[6]);
            }else if(mode  == e_mode::in_Velocity){
                HYYRobotBase::SetGroupVelocity(addGroups[i].component_name, &(addGroups[i].joint_velocity_command_[0]));
                // RCLCPP_INFO(logger_, "read addgroup(%d) joints velocity: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addgroups[i].joint_velocity_command_[0],
                //             addgroups[i].joint_velocity_command_[1], addgroups[i].joint_velocity_command_[2], addgroups[i].joint_velocity_command_[3],
                //             addgroups[i].joint_velocity_command_[4], addgroups[i].joint_velocity_command_[5], addgroups[i].joint_velocity_command_[6]);
            }else if (mode == e_mode::in_Effort){
                HYYRobotBase::SetGroupTorque(addGroups[i].component_name, &(addGroups[i].joint_effort_command_[0]));
                // RCLCPP_INFO(logger_, "read addgroup(%d) joints effort: %f\t%f\t%f\t%f\t%f\t%f\t%f.", i, addgroups[i].joint_effort_command_[0],
                //             addgroups[i].joint_effort_command_[1], addgroups[i].joint_effort_command_[2], addgroups[i].joint_effort_command_[3],
                //             addgroups[i].joint_effort_command_[4], addgroups[i].joint_effort_command_[5], addgroups[i].joint_effort_command_[6]);
            }
        }
    }
    if (if_add_external_device){
        externalDevices.joint_position_state_ = externalDevices.joint_position_command_;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace hyy_hardware_interface

PLUGINLIB_EXPORT_CLASS(hyy_hardware_interface::HyyHardwareInterface, hardware_interface::SystemInterface)
