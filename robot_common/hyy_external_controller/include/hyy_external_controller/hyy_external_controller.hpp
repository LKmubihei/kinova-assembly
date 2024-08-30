#ifndef HYY_EXTERNAL_CONTROLLER__HYY_EXTERNAL_CONTROLLER_HPP_
#define HYY_EXTERNAL_CONTROLLER__HYY_EXTERNAL_CONTROLLER_HPP_

#include "hyy_external_controller/visibility_control.h"
#include "hyy_external_controller_parameters.hpp"
#include <algorithm>
#include <utility>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "pluginlib/class_list_macros.hpp"

#include "serial/serial.h"

#include "hyy_message/srv/setangle.hpp"
#include "hyy_message/srv/getangleact.hpp"
#include "hyy_message/srv/setpos.hpp"
#include "hyy_message/srv/setspeed.hpp"
#include "hyy_message/srv/setforce.hpp"
#include "hyy_message/srv/getangleset.hpp"
#include "hyy_message/srv/getposact.hpp"
#include "hyy_message/srv/getposset.hpp"
#include "hyy_message/srv/getspeedset.hpp"
#include "hyy_message/srv/getforceact.hpp"
#include "hyy_message/srv/getforceset.hpp"
#include "hyy_message/srv/getcurrentact.hpp"
#include "hyy_message/srv/geterror.hpp"
#include "hyy_message/srv/gettemp.hpp"
#include "hyy_message/srv/robotgrip.hpp"

#include "hyy_external_controller/hyy_system.h"

namespace hyy_external_controller
{ 

  static unsigned char highByteLookup[] = {
      0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
      0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
      0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
      0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
      0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
      0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
      0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
      0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
      0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
      0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
      0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
      0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
      0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
      0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
      0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
      0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
      0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
      0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
      0xC1, 0x81, 0x40};

  static unsigned char lowByteLookup[] = {
      0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07,
      0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF,
      0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8,
      0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
      0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16,
      0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30,
      0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5,
      0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
      0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9,
      0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
      0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22,
      0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
      0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64,
      0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A,
      0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
      0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
      0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3,
      0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53,
      0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
      0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
      0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A,
      0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84,
      0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41,
      0x81, 0x80, 0x40};

  using  hyySetangleMsg = hyy_message::srv::Setangle;
  using  hyySetposMsg = hyy_message::srv::Setpos;
  using  hyySetspeedMsg = hyy_message::srv::Setspeed;
  using  hyySetforceMsg = hyy_message::srv::Setforce;
  using  hyyGetangleactMsg = hyy_message::srv::Getangleact;
  using  hyyGetanglesetMsg = hyy_message::srv::Getangleset;
  using  hyyGetposactMsg = hyy_message::srv::Getposact;
  using  hyyGetpossetMsg = hyy_message::srv::Getposset;
  using  hyyGetspeedsetMsg = hyy_message::srv::Getspeedset;
  using  hyyGetforceactMsg = hyy_message::srv::Getforceact;
  using  hyyGetforcesetMsg = hyy_message::srv::Getforceset;
  using  hyyGetcurrentactMsg = hyy_message::srv::Getcurrentact;
  using  hyyGeterrorMsg = hyy_message::srv::Geterror;
  using  hyyGettempMsg = hyy_message::srv::Gettemp;
  using  hyyGripMsg = hyy_message::srv::Robotgrip;

  class HyyExternalController : public controller_interface::ControllerInterface
  {

    /***********************
    * \brief 设置灵巧手角度

    ***********************/
  public:

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    HyyExternalController();

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    ~HyyExternalController();

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    HYY_EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  
  private:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    bool start_controller;
    int if_add_force_sensor;
    int if_add_hand;
    int if_add_gripper;

  protected:
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void setangle_callback(const hyySetangleMsg::Request::SharedPtr request,
                           const hyySetangleMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void setpos_callback(const hyySetposMsg::Request::SharedPtr request,
                         const hyySetposMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void setspeed_callback(const hyySetspeedMsg::Request::SharedPtr request,
                           const hyySetspeedMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void setforce_callback(const hyySetforceMsg::Request::SharedPtr request,
                           const hyySetforceMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getangleact_callback(const hyyGetangleactMsg::Request::SharedPtr request,
                              const hyyGetangleactMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getangleset_callback(const hyyGetanglesetMsg::Request::SharedPtr request,
                              const hyyGetanglesetMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getposact_callback(const hyyGetposactMsg::Request::SharedPtr request,
                            const hyyGetposactMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getposset_callback(const hyyGetpossetMsg::Request::SharedPtr request,
                            const hyyGetpossetMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getspeedset_callback(const hyyGetspeedsetMsg::Request::SharedPtr request,
                              const hyyGetspeedsetMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getforceact_callback(const hyyGetforceactMsg::Request::SharedPtr request,
                              const hyyGetforceactMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getforceset_callback(const hyyGetforcesetMsg::Request::SharedPtr request,
                              const hyyGetforcesetMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void getcurrentact_callback(const hyyGetcurrentactMsg::Request::SharedPtr request,
                                const hyyGetcurrentactMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void geterror_callback(const hyyGeterrorMsg::Request::SharedPtr request,
                           const hyyGeterrorMsg::Response::SharedPtr response);
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void gettemp_callback(const hyyGettempMsg::Request::SharedPtr request,
                          const hyyGettempMsg::Response::SharedPtr response);

  private:
    std::vector<std::string> sensors_name;
    std::string port_hand;
    int baudrate_hand;
    serial::Serial ros_ser;
    unsigned char send_buffer[64];
    unsigned char recv_buffer[64];
    std::vector<double> hand_joints_position;

    rclcpp::CallbackGroup::SharedPtr callback_group_setangle;
    rclcpp::CallbackGroup::SharedPtr callback_group_setpos;
    rclcpp::CallbackGroup::SharedPtr callback_group_setspeed;
    rclcpp::CallbackGroup::SharedPtr callback_group_setforce;
    rclcpp::CallbackGroup::SharedPtr callback_group_getangleact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getangleset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getposact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getposset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getspeedset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getforceact;
    rclcpp::CallbackGroup::SharedPtr callback_group_getforceset;
    rclcpp::CallbackGroup::SharedPtr callback_group_getcurrentact;
    rclcpp::CallbackGroup::SharedPtr callback_group_geterror;
    rclcpp::CallbackGroup::SharedPtr callback_group_gettemp;

    rclcpp::Service<hyySetangleMsg>::SharedPtr Setangle_Server;
    rclcpp::Service<hyySetposMsg>::SharedPtr Setpos_Server;
    rclcpp::Service<hyySetspeedMsg>::SharedPtr Setspeed_Server;
    rclcpp::Service<hyySetforceMsg>::SharedPtr Setforce_Server;
    rclcpp::Service<hyyGetangleactMsg>::SharedPtr Getangleact_Server;
    rclcpp::Service<hyyGetanglesetMsg>::SharedPtr Getangleset_Server;
    rclcpp::Service<hyyGetposactMsg>::SharedPtr Getposact_Server;
    rclcpp::Service<hyyGetpossetMsg>::SharedPtr Getposset_Server;
    rclcpp::Service<hyyGetspeedsetMsg>::SharedPtr Getspeedset_Server;
    rclcpp::Service<hyyGetforceactMsg>::SharedPtr Getforceact_Server;
    rclcpp::Service<hyyGetforcesetMsg>::SharedPtr Getforceset_Server;
    rclcpp::Service<hyyGetcurrentactMsg>::SharedPtr Getcurrentact_Server;
    rclcpp::Service<hyyGeterrorMsg>::SharedPtr Geterror_Server;
    rclcpp::Service<hyyGettempMsg>::SharedPtr Gettemp_Server;

    /***********************
    * \brief 存储夹爪状态
    ***********************/
    double store_gripper_state(int position);

    /***********************
    * \brief 存储灵巧手状态
    ***********************/
    std::vector<double> store_hand_state(std::vector<int> position);

  public:

    // 写入事件
    enum class CommandTopic
    {
        Activation,
        Move,
        RequestedPosition,
        Speed,
        Force
    };
    // 读入事件
    enum class QueryTopic
    {
        Activation,
        Status
    };
    // 写寄存器地址
    enum class writeRegisterAddress
    {
        ActionRequest = 0x00,
        PositionRequest = 0x03,
        Speed = 0x04,
        Force = 0x05
    };
    // 读寄存器地址
    enum class readRegisterAddress
    {
        GripperStatus = 0x00,
        FaultStatus = 0x02,
        RequestedPosition = 0x03,
        Position = 0x04,
        Current = 0x05,
    };
    // Action寄存器位操作
    enum class ActionRequestBitField
    {
        Activation = 0x00,
        GoTo = 0x03,
        EmergencyRelease = 0x04,
        EmergencyReleaseDirection = 0x05
    };
    
    void SendCommand(CommandTopic property, uint8_t value);
    
    void sendQuery(QueryTopic property);

    void setBits(uint8_t address, int pos, uint8_t value);
    
    unsigned short checksum(const std::vector<uint8_t> &message);

  protected:
  
    HYY_EXTERNAL_CONTROLLER_PUBLIC
    void controlgripper_callback(const hyyGripMsg::Request::SharedPtr request,
                          const hyyGripMsg::Response::SharedPtr response);

  private: 

    rclcpp::CallbackGroup::SharedPtr callback_group_controlgripper;
    rclcpp::Service<hyyGripMsg>::SharedPtr ControlGripper_Server;

    // 全局变量寄存器，共三个寄存器，一个寄存器两个字节，所以共六个字节，初始值设为0
    uint8_t dataRegister[6] = {0};
    const uint8_t slaveId = 0x09;                          // 从站设备ID
    const uint8_t writeRegisterCode = 0x10;                // 写寄存器功能码
    const uint8_t readRegisterCode = 0x04;                 // 读寄存器功能码
    const uint16_t writeRegistersStart = 0x03 << 8 | 0xE8; // 写寄存器起始地址
    const uint16_t readRegistersStart = 0x07 << 8 | 0xD0;  // 读寄存器起始地址
    uint16_t registerNumber = 3;
    int sendflag = 0;
    unsigned short crcValue = 0;
    std::vector<uint8_t> modbus_write;
    std::vector<uint8_t> modbus_read;

    serial::Serial gripper_ser;

    std::string port_gripper;
    int baudrate_gripper;

    double gripper_joint_degree;

    std::mutex hand_lock, gripper_lock;
  };

}  // namespace hyy_external_controller

#endif  // HYY_EXTERNAL_CONTROLLER__HYY_EXTERNAL_CONTROLLER_HPP_
