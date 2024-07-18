#ifndef HYY_ROBOT_CONTROL_INCLUDE_HYY_ROBOT_CONTROL_HYY_ROBOT_CONTROL_H_
#define HYY_ROBOT_CONTROL_INCLUDE_HYY_ROBOT_CONTROL_HYY_ROBOT_CONTROL_H_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <mutex>
#include "hyy_message/srv/robotmove.hpp"
#include "hyy_message/srv/robotgrip.hpp"
#include "hyy_message/srv/robotmovedata.hpp"
#include "hyy_message/srv/robotio.hpp"
#include "hyy_message/srv/robotgeneralcontrol.hpp"

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

extern std::atomic<bool> stop;

namespace hyy_robot_control
{
    
#define R_PI 3.1415926535898

using hyyMoveMsg = hyy_message::srv::Robotmove;
using hyyIoMsg = hyy_message::srv::Robotio;
using hyyMoveDataMsg = hyy_message::srv::Robotmovedata;
using hyyGripMsg = hyy_message::srv::Robotgrip;
using hyyGeneralControlMsg = hyy_message::srv::Robotgeneralcontrol;

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

class HyyRobotControl{    

public:
    HyyRobotControl(std::shared_ptr<rclcpp::Node> node);
    HyyRobotControl();

    /***********************
    * \brief 用户接口初始化
    * \param controller_name 控制器名
    * \param type 控制器类型（0-机械臂和轴组控制器，1-外部设备控制器）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool init(std::string controller_name, int type);

    /***********************
    * \brief 机械臂或附加轴组做绝对关节位置运动指令（控制器中的目标）
    * @param target 期望关节位置    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveA(const std::string &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");

    /***********************
    * \brief 机械臂或附加轴组做绝对关节位置运动指令（程序自定目标）
    * @param target 期望关节位置    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveA(std::vector<double> &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
    
    /***********************
    * \brief 机械臂做绝对笛卡尔位置运动指令（控制器中的目标）（关节层面规划）
    * @param target 期望位置（位姿）    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveJ(const std::string &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");

    /***********************
    * \brief 机械臂做绝对笛卡尔位置运动指令（程序自定目标）（关节层面规划）
    * @param target 期望位置（位姿）    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveJ(std::vector<double> &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
    
    /***********************
    * \brief 机械臂做绝对笛卡尔位置运动指令（控制器中的目标）（笛卡尔层面规划）
    * @param target 期望位置（位姿）    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveL(const std::string &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");

    /***********************
    * \brief 机械臂做绝对笛卡尔位置运动指令（程序自定目标）（笛卡尔层面规划）
    * @param target 期望位置（位姿）    单位：rad
    * @param velocity 运动速度  单位：rad/s
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveL(std::vector<double> &target, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
    
    /***********************
    * \brief 机械臂做圆弧运动指令（控制器中的目标）
    * @param target 期望位置（位姿）
    * @param target_mid 途经点期望位置（位姿）
    * @param velocity 运动速度
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveC(const std::string &target, const std::string &target_mid, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
    
    /***********************
    * \brief 机械臂做圆弧运动指令（程序自定目标）
    * @param target 期望位置（位姿）
    * @param target_mid 途经点期望位置（位姿）
    * @param velocity 运动速度
    * @param zone 转动空间
    * @param tool 末端工具
    * @param wobj 坐标系
    * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
    ***********************/
    int moveC(std::vector<double> &target, std::vector<double> &target_mid, const std::string &velocity = "DEFAULT_SPEED", const std::string &zone = "DEFAULT_ZONE", const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
	
    /**
     * @brief 获取数字量输入
     * @param id 数字量输入索引号
     * @return int 返回值0或1
     */
    int getDI(int index);

    /**
     * @brief 设置数字量输出
     * @param id 数字量输出索引号
     * @param flag 0或1
     */
    int setDO(int index, bool value);
	
    /**
     * @brief 从系统获取关节位置
     * @param name 数据名字
     * @return 关节位置
     */
    std::vector<double> get_joint_data(const std::string& name);

    /**
     * @brief 从系统获取笛卡尔位姿
     * @param name 数据名字
     * @return 笛卡尔位姿
     */
    std::vector<double> get_cartesian_data(const std::string& name);

    /**
     * @brief 从系统获取关节位置
     * @return 关节位置
     */
    std::vector<double> get_joint_current(void);

    /**
     * @brief 获取当前笛卡尔位姿
     * @param tool 工具坐标系
     * @param wobj 工作坐标系
     * @return 当前笛卡尔位姿
     */
    std::vector<double> get_cartesian_current(const std::string &tool = "DEFAULT_TOOL", const std::string &wobj = "DEFAULT_WOBJ");
    
    /**
     * @brief 设置控制器运行模式
     * @param value 0-非堵塞运动（开始运动后可以执行后面的命令） 1-堵塞运动 （开始运动后等待该运动完毕）
     * @return 无
     */
    void isblock(bool value);

    /**
     * @brief 全局坐标系中位姿偏移
     * @param target 目标位姿
     * @param xyzkps 偏移量
     * @return 偏移后的位姿
     */
    std::vector<double> offs(std::vector<double>& target, double x, double y, double z, double k, double p, double s);
	
    /**
     * @brief 当前姿态应用偏移
     * @param target 目标位姿
     * @param xyzkps 偏移量
     * @return 偏移后的位姿
     */
    std::vector<double> offsrel(std::vector<double>& target, double x, double y, double z, double k, double p, double s);
	
    /**
     * @brief 创建夹爪
     * @param name 夹爪名称
     * @return 0-创建成功 失败返回其他 
     */
    int grip_create(const std::string& name);

    /**
     * @brief 控制夹爪运动
     * @param name 夹爪名称
     * @param position 期望夹爪位置
     * @return 0-运动成功 失败返回其他 
     */
    int grip_control(const std::string& name,double position);

    /**
     * @brief 注销夹爪
     * @param name 夹爪名称
     * @return 0-注销成功 失败返回其他 
     */
    int grip_destroy(const std::string& name);

    /**
     * @brief 询问控制器运行状态
     * @return int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
     */
    int ask_status();

    /**
     * @brief 停止所有子设备运行，不下电，可继续执行后续命令
     * @return int 0：正常，<0:异常
     */
    int stopDeviceRun();

    /**
     * @brief 停止机器人运行，不下电，可继续执行后续命令
     * @return int 0：正常，<0:异常
     */
    int stopRobotRun();

    /**
     * @brief 停止附加轴组运行，不下电，可继续执行后续命令
     * @return int 0：正常，<0:异常
     */
    int stopAddaxisRun();

    /**
     * @brief 机器人状态是否正常
     * @return int 1:状态正常; 0:状态错误或强制退出
     */
    int robot_ok();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string MoveSrvName_, IoSrvName_, MoveDataSrvName_, GripSrvName_, GeneralControlSrvName_;
    rclcpp::Client<hyyMoveMsg>::SharedPtr robotMoveClient;
    rclcpp::Client<hyyIoMsg>::SharedPtr robotIoClient;
    rclcpp::Client<hyyMoveDataMsg>::SharedPtr robotMoveDataClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr robotGripClient;
    rclcpp::Client<hyyGeneralControlMsg>::SharedPtr robotGeneralControlClient;
    std::shared_ptr<hyyMoveMsg::Request> moveReq;
    std::shared_ptr<hyyIoMsg::Request> ioReq;
    std::shared_ptr<hyyMoveDataMsg::Request> moveDataReq;
    std::shared_ptr<hyyGripMsg::Request> gripReq;
    std::shared_ptr<hyyGeneralControlMsg::Request> generalControlReq;
    bool init_flag, block_flag;

    int wait_move_finish(rclcpp::Client<hyy_robot_control::hyyMoveMsg>::FutureAndRequestId &resp);
    void rpy2tr(double *rpy, double R[3][3], int flag);
    void tr2rpy(double R[3][3], double* rpy, int flag);
	void Rmulti(double R0[3][3],double R1[3][3],double Rres[3][3]);
	void RMultVec(double(*R)[3], double* v, double * vres);

public:

    /***********************
    * \brief 设置灵巧手角度
    * \param angle 角度数据（范围0-1000）
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool hand_SetAngle(std::vector<int> angle, const int hand_id = 1);

    /***********************
    * \brief 设置灵巧手角度（全张开-角度1000）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool hand_fullopen();

    /***********************
    * \brief 设置灵巧手力控阈值
    * \param force 阀值（范围0-1000）
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool hand_SetForce(std::vector<int> force, const int hand_id = 1);

    /***********************
    * \brief 设置灵巧手驱动器位置
    * \param pos 驱动器位置数据（范围0-2000）
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool hand_SetPos(std::vector<int> pos, const int hand_id = 1);
    
    /***********************
    * \brief 设置灵巧手速度
    * \param speed 速度（范围0-1000）
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool hand_SetSpeed(std::vector<int> speed, const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手实际的角度值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 角度数据
    ***********************/
    std::vector<int> hand_GetAngleAct(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手设定的角度值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 角度数据
    ***********************/
    std::vector<int> hand_GetAngleSet(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手当前电流值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 电流数据
    ***********************/
    std::vector<int> hand_GetCurrentAct(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手故障代码
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 故障代码数据
    ***********************/
    std::vector<int> hand_GetError(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手实际的受力
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 受力数据
    ***********************/
    std::vector<int> hand_GetForceAct(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手设定的力控阀值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 阀值数据
    ***********************/
    std::vector<int> hand_GetForceSet(const int hand_id = 1);

    /***********************
    * \brief 读取驱动器实际的位置值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 驱动器实际的位置数据
    ***********************/
    std::vector<int> hand_GetPosAct(const int hand_id = 1);

    /***********************
    * \brief 读取驱动器设定的位置值
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 驱动器设定的位置数据
    ***********************/
    std::vector<int> hand_GetPosSet(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手设定的速度
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 速度数据
    ***********************/
    std::vector<int> hand_GetSpeedSet(const int hand_id = 1);

    /***********************
    * \brief 读取灵巧手温度
    * \param hand_id 灵巧手编号（默认为 1）
    * \return 温度数据
    ***********************/
    std::vector<int> hand_GetTemp(const int hand_id = 1);

private:
    std::string SetangleSrvName_, SetposSrvName_, SetspeedSrvName_, SetforceSrvName_;
    std::string GetangleactSrvName_, GetanglesetSrvName_, GetposactSrvName_, GetpossetSrvName_;
    std::string GetspeedsetSrvName_, GetforceactSrvName_, GetforcesetSrvName_, GetcurrentactSrvName_;
    std::string GeterrorSrvName_, GettempSrvName_;

    rclcpp::Client<hyySetangleMsg>::SharedPtr hyySetangleClient;
    rclcpp::Client<hyySetposMsg>::SharedPtr hyySetposClient;
    rclcpp::Client<hyySetspeedMsg>::SharedPtr hyySetspeedClient;
    rclcpp::Client<hyySetforceMsg>::SharedPtr hyySetforceClient;
    rclcpp::Client<hyyGetangleactMsg>::SharedPtr hyyGetangleactClient;
    rclcpp::Client<hyyGetanglesetMsg>::SharedPtr hyyGetanglesetClient;
    rclcpp::Client<hyyGetposactMsg>::SharedPtr hyyGetposactClient;
    rclcpp::Client<hyyGetpossetMsg>::SharedPtr hyyGetpossetClient;
    rclcpp::Client<hyyGetspeedsetMsg>::SharedPtr hyyGetspeedsetClient;
    rclcpp::Client<hyyGetforceactMsg>::SharedPtr hyyGetforceactClient;
    rclcpp::Client<hyyGetforcesetMsg>::SharedPtr hyyGetforcesetClient;
    rclcpp::Client<hyyGetcurrentactMsg>::SharedPtr hyyGetcurrentactClient;
    rclcpp::Client<hyyGeterrorMsg>::SharedPtr hyyGeterrorClient;
    rclcpp::Client<hyyGettempMsg>::SharedPtr hyyGettempClient;

    std::shared_ptr<hyySetangleMsg::Request> SetangleReq;
    std::shared_ptr<hyySetposMsg::Request> SetposReq;
    std::shared_ptr<hyySetspeedMsg::Request> SetspeedReq;
    std::shared_ptr<hyySetforceMsg::Request> SetforceReq;
    std::shared_ptr<hyyGetangleactMsg::Request> GetangleactReq;
    std::shared_ptr<hyyGetanglesetMsg::Request> GetanglesetReq;
    std::shared_ptr<hyyGetposactMsg::Request> GetposactReq;
    std::shared_ptr<hyyGetpossetMsg::Request> GetpossetReq;
    std::shared_ptr<hyyGetspeedsetMsg::Request> GetspeedsetReq;
    std::shared_ptr<hyyGetforceactMsg::Request> GetforceactReq;
    std::shared_ptr<hyyGetforcesetMsg::Request> GetforcesetReq;
    std::shared_ptr<hyyGetcurrentactMsg::Request> GetcurrentactReq;
    std::shared_ptr<hyyGeterrorMsg::Request> GeterrorReq;
    std::shared_ptr<hyyGettempMsg::Request> GettempReq;

    std::vector<int> empty;

public:

    /***********************
    * \brief 设置夹爪位置（不运动）
    * \param pos 角度数据（范围0-255)
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_SetPos(const int pos);

    /***********************
    * \brief 设置夹爪运动速度
    * \param speed 角度数据（范围0-255)
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_SetSpeed(const int speed);

    /***********************
    * \brief 设置夹爪夹紧力度
    * \param pos 角度数据（范围0-255)
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_SetForce(const int force);

    /***********************
    * \brief 夹爪运动
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_Move();

    /***********************
    * \brief 夹爪运动指定位置
    * \param pos 角度数据（范围0-255)
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_goto(const int pos);

    /***********************
    * \brief 夹爪全开
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_fullopen();

    /***********************
    * \brief 夹爪全关
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_fullclose();

    /***********************
    * \brief 激活夹爪
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_Activate();

    /***********************
    * \brief 禁用夹爪
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_Deactivate();

    /***********************
    * \brief 初始化夹爪
    * \return 成功返回 true ,其他返回 false
    ***********************/
    bool Gripper_initialize();

    /***********************
    * \brief 获得夹爪激活状态
    * \return 
    ***********************/
    int Gripper_GetStatus();

    /***********************
    * \brief 获得夹爪当前位置
    * \return 
    ***********************/
    int Gripper_GetPos();

private:

    std::string SetGripPosSrvName_;
    std::string SetGripSpeedSrvName_;
    std::string SetGripForceSrvName_;
    std::string GripMoveSrvName_;
    std::string ActivateGripSrvName_;
    std::string DeactivateGripSrvName_;
    std::string GetGripStatusSrvName_;
    std::string GetGripPosSrvName_;

    rclcpp::Client<hyyGripMsg>::SharedPtr hyySetGripPosClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyySetGripSpeedClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyySetGripForceClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyyGripMoveClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyyActivateGripClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyyDeactivateGripClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyyGetGripStatusClient;
    rclcpp::Client<hyyGripMsg>::SharedPtr hyyGetGripPosClient;

    std::shared_ptr<hyyGripMsg::Request> SetGripPosReq;
    std::shared_ptr<hyyGripMsg::Request> SetGripSpeedReq;
    std::shared_ptr<hyyGripMsg::Request> SetGripForceReq;
    std::shared_ptr<hyyGripMsg::Request> GripMoveReq;
    std::shared_ptr<hyyGripMsg::Request> ActivateGripReq;
    std::shared_ptr<hyyGripMsg::Request> DeactivateGripReq;
    std::shared_ptr<hyyGripMsg::Request> GetGripStatusReq;
    std::shared_ptr<hyyGripMsg::Request> GetGripPosReq;

};

} // namespace hyy_robot_control

#endif