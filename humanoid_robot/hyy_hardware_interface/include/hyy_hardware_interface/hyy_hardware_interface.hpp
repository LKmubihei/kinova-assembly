#ifndef HYY_HARDWARE_INTERFACE__HYY_HARDWARE_INTERFACE_HPP_
#define HYY_HARDWARE_INTERFACE__HYY_HARDWARE_INTERFACE_HPP_

#include "hyy_hardware_interface/visibility_control.h"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <fstream>
#include <iomanip>
#include <string>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <urdf/model.h>
#include <joint_limits/joint_limits.hpp>
#include "device_interface/Tool/saveData.h"
namespace hyy_hardware_interface
{

  enum class e_mode
  {
    in_Position,
    in_Velocity,
    in_Effort
  };

  struct Component_Imformation
  {

    const char *component_name;           // Name
    size_t _dof;                          // Degree of Freedom
    signed char device_mode; // Device Mode of Actuator
    std::vector<std::string> joints;      // Every Joint Name
    int _index;

    // Inagurate the store memory for Joint State in reading and writing
    std::vector<double> joint_position_state_;
    std::vector<double> joint_velocity_state_;
    std::vector<double> joint_effort_state_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;

    // // The JointHandle for commanding
    // std::vector<hardware_interface::StateInterface> state_interfaces;
    // std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Joint Limit Value;
    // std::vector<joint_limits::JointLimits> joint_limit_;
    // std::vector<joint_limits::SoftJointLimits> joint_soft_limit_;

  };

  class HyyHardwareInterface : public hardware_interface::SystemInterface
  {
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(HyyHardwareInterface)

      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      HYY_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      e_mode convertToEMode(signed char value);

      HyyHardwareInterface();
      virtual ~HyyHardwareInterface();

      private:
        std::vector<double> joint_state_callback();

        std::vector<std::string> joints; 
        std::vector<Component_Imformation> robots; //robots information: left_arm(0),right_arm(1)
        std::vector<Component_Imformation> addGroups; //addGroups information: left_arm(0),right_arm(1)
        Component_Imformation externalDevices; //externalDevices information
        int robots_num;
        int addGroups_num;
        bool _sim_flag;
        bool if_add_axisgroups;
        bool if_add_external_device;
        int e_m;
        e_mode mode;
        rclcpp::Logger logger_;
  };

}  // namespace hyy_hardware_interface

#endif  // HYY_HARDWARE_INTERFACE__HYY_HARDWARE_INTERFACE_HPP_
