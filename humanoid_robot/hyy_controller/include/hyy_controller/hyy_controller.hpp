#ifndef HYY_CONTROLLER__HYY_CONTROLLER_HPP_
#define HYY_CONTROLLER__HYY_CONTROLLER_HPP_

#include "hyy_controller/visibility_control.h"
#include <algorithm>
#include <utility>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "pluginlib/class_list_macros.hpp"

#include "device_interface/DeviceDriver/device_interface.h"
#include "device_interface/Move/MovePlan.h"
#include "device_interface/Base/RobotStruct.h"
#include "device_interface/Grip/grip_interface.h"

#include "hyy_message/srv/robotmove.hpp"
#include "hyy_message/srv/robotgrip.hpp"
#include "hyy_message/srv/robotmovedata.hpp"
#include "hyy_message/srv/robotio.hpp"

#include "hyy_controller_parameters.hpp"

#include "hyy_controller/hyy_system.h"

namespace hyy_controller
{

  using hyyMoveMsg = hyy_message::srv::Robotmove;
  using hyyGripMsg = hyy_message::srv::Robotgrip;
  using hyyMoveDataMsg = hyy_message::srv::Robotmovedata;
  using hyyIoMsg = hyy_message::srv::Robotio;

  class HyyController : public controller_interface::ControllerInterface
  {
  public:
    HYY_CONTROLLER_PUBLIC
    HyyController();

    HYY_CONTROLLER_PUBLIC
    ~HyyController();

    HYY_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    HYY_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;


  protected:

    HYY_CONTROLLER_PUBLIC
    void robotmove_command_callback(const std::shared_ptr<hyy_message::srv::Robotmove::Request> req,
                                    std::shared_ptr<hyy_message::srv::Robotmove::Response> res);

    HYY_CONTROLLER_PUBLIC
    void robotio_command_callback(const std::shared_ptr<hyy_message::srv::Robotio::Request> req,
                                  std::shared_ptr<hyy_message::srv::Robotio::Response> res);

    HYY_CONTROLLER_PUBLIC
    void robotmovedata_command_callback(const std::shared_ptr<hyy_message::srv::Robotmovedata::Request> req,
                                    std::shared_ptr<hyy_message::srv::Robotmovedata::Response> res);

    HYY_CONTROLLER_PUBLIC
    void robotgrip_command_callback(const std::shared_ptr<hyy_message::srv::Robotgrip::Request> req,
                                    std::shared_ptr<hyy_message::srv::Robotgrip::Response> res);

    HYYRobotBase::robjoint rjoint_;
    HYYRobotBase::robpose rpose_;
    HYYRobotBase::robpose rpose_mid_;
    HYYRobotBase::speed rspeed_;
    HYYRobotBase::zone rzone_;
    HYYRobotBase::tool rtool_;
    HYYRobotBase::wobj rwobj_;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    int component_index_;
    int dof_;
    const char* component_name_;
    std::vector<std::string> joints_;
    bool start_controller;
    bool block_flag;
    int if_additionaxis;

    rclcpp::Service<hyyMoveMsg>::SharedPtr hyyMoveSrv;
    rclcpp::Service<hyyGripMsg>::SharedPtr hyyGripSrv;
    rclcpp::Service<hyyMoveDataMsg>::SharedPtr hyyMoveDataSrv;
    rclcpp::Service<hyyIoMsg>::SharedPtr hyyIoSrv;

    double velocity_data_type(const std::string &velocity);

  private:
    double *joint_speed;
    double *default_tool_frame;
    double *default_userframe;
    double *default_workframe;
    double *default_addframe;
    HYYRobotBase::PayLoad default_payload;
  };

}  // namespace hyy_controller

#endif  // HYY_CONTROLLER__HYY_CONTROLLER_HPP_
