#ifndef XMATE_HARDWARE_INTERFACE__XMATE_HARDWARE_INTERFACE_HPP_
#define XMATE_HARDWARE_INTERFACE__XMATE_HARDWARE_INTERFACE_HPP_

#include "xmate_hardware_interface/visibility_control.h"
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

namespace xmate_hardware_interface
{

  enum class e_mode
  {
    in_Position = 8,
    in_Velocity = 9,
    in_Effort = 10
  };

  struct Component_Imformation
  {

    const char *component_name;           // Name
    size_t _dof;                          // Degree of Freedom
    signed char device_mode; // Device Mode of Actuator
    std::vector<std::string> joints;      // Every Joint Name
    size_t joints_num;
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

  class XmateHardwareInterface : public hardware_interface::SystemInterface
  {
    public:

      RCLCPP_SHARED_PTR_DEFINITIONS(XmateHardwareInterface)

      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      XMATE_HARDWARE_INTERFACE_PUBLIC
      hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      XmateHardwareInterface();
      virtual ~XmateHardwareInterface();
      e_mode convertToEMode(signed char value);
    
    private:
      Component_Imformation robot; //bodyGroups information:
      bool _sim_flag;
      int e_m;
      e_mode mode;
      rclcpp::Logger logger_;
      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;    
};

}  // namespace xmate_hardware_interface

#endif  // XMATE_HARDWARE_INTERFACE__XMATE_HARDWARE_INTERFACE_HPP_
