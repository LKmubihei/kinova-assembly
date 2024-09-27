#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include "hyy_message/action/move_xyzw.hpp"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>  

/***************************************************************************/
/*                                                                         */
/*   Standard application structure   Client Initialize!!                  */
/*                                                                         */
/***************************************************************************/

std::string control_group = "none";
std::string base_frame = "none";
moveit::planning_interface::MoveGroupInterface move_group_interface;

/***************************************************************************/
/*                                                                         */
/*   Standard application structure   Client Initialize!!                  */
/*                                                                         */
/***************************************************************************/

class ParamServer : public rclcpp::Node
{
    public:
        ParamServer() : Node("param_server") 
        {
            this->declare_parameter("control_group", "null");
            control_group = this->get_parameter("control_group").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "control_group received -> %s", control_group.c_str());
            
            this->declare_parameter("base_frame", "null");
            base_frame = this->get_parameter("base_frame").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "base_frame received -> %s", base_frame.c_str());
        }
    private:
};

/***************************************************************************/
/*                                                                         */
/*   Standard application structure   Client Initialize!!                  */
/*                                                                         */
/***************************************************************************/

class ActionServer : public rclcpp::Node
{
    public:
        using MoveXYZW = hyy_message::action::MoveXYZW;
        using GoalHandle = rclcpp_action::ServerGoalHandle<MoveXYZW>;

        explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("MoveXYZW_ActionServer", options)
        {
            action_server_ = rclcpp_action::create_server<MoveXYZW>(
                this,
                "/MoveXYZW",
                std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
        }

    private:
        rclcpp_action::Server<MoveXYZW>::SharedPtr action_server_;
        
        // 检查接收到的目标，并相应地接受它
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveXYZW::Goal> goal)
        {
            double positionX =  goal->positionx;
            double positionY =  goal->positiony;
            double positionZ =  goal->positionz;
            double yaw =  goal->yaw;
            double pitch =  goal->pitch;
            double roll =  goal->roll;
            RCLCPP_INFO(get_logger(), "Received a POSE GOAL request:");
            RCLCPP_INFO(this->get_logger(), "POSITION -> (x = %.2f, y = %.2f, z = %.2f)", positionX, positionY, positionZ);
            RCLCPP_INFO(this->get_logger(), "ORIENTATION (euler) -> (yaw = %.2f, pitch = %.2f, roll = %.2f)", yaw, pitch, roll);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 接受并执行接收到的目标
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            // 为了避免阻塞执行器，需要快速返回，所以启动一个新线程
            std::thread(
                [this, goal_handle]() {
                    execute(goal_handle);
                }).detach();
        }

        // 取消目标请求的函数
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

            // 调用 MoveGroupInterface 的 stop() 方法，如果有活动的轨迹执行，则停止它
            move_group_interface.stop();

            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Action Server 的主循环 -> 执行
        void execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Starting MoveXYZW motion to desired waypoint...");
            auto result = std::make_shared<MoveXYZW::Result>();
            
            std::vector<geometry_msgs::msg::Pose> waypoints;

            const auto goal = goal_handle->get_goal();
            geometry_msgs::msg::Pose pose;
            pose.position.x = goal->positionx;
            pose.position.y = goal->positiony;
            pose.position.z = goal->positionz;

            tf2::Quaternion q;
            q.setRPY(goal->roll, goal->pitch, goal->yaw);
            pose.orientation = tf2::toMsg(q);

            waypoints.push_back(pose);

            // 获取关节速度并应用到 MoveIt!2
            auto speed_ = goal->speed;
            auto accel_ = goal->accel;
            move_group_interface.setPoseReferenceFrame(base_frame); 
            move_group_interface.setStartStateToCurrentState();
            
            moveit_msgs::msg::RobotTrajectory trajectory;
            double eef_step = 0.005; // 末端执行器步长（米）
            double jump_threshold = 0; // 跳跃阈值，设置为 0 以禁用跳跃检测
            double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            // 检查规划完成的比例
            if (fraction < 0.99) {
                RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the planned path has been completed, planning may fail.", fraction * 100.0);
            }

            robot_trajectory::RobotTrajectory rt(move_group_interface.getRobotModel(), move_group_interface.getName());
            rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(rt, speed_, accel_);
            rt.getRobotTrajectoryMsg(trajectory);

            // 规划、执行并反馈信息
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;

            bool success = (move_group_interface.execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: Cartesian path execution successful! ", control_group.c_str());
                result->result = "MoveXYZW:SUCCESS";
                goal_handle->succeed(result);
            } else {
                RCLCPP_WARN(this->get_logger(), "%s - MoveXYZW: Cartesian path execution failed! ", control_group.c_str());
                result->result = "MoveXYZW:FAILED";
                goal_handle->succeed(result);
            } 
        }
};

/***************************************************************************/
/*                                                                         */
/*   Standard application structure   Client Initialize!!                  */
/*                                                                         */
/***************************************************************************/

int main(int argc, char ** argv)
{
    // 初始化主节点
    rclcpp::init(argc, argv);

    // 获取 ParamServer 参数
    auto node_PARAM = std::make_shared<ParamServer>();
    rclcpp::spin_some(node_PARAM);

    // 启动并运行（执行器）MoveIt!2 接口节点
    auto name = "_MoveXYZW_interface";
    auto node2name = control_group + name;
    auto const node2 = std::make_shared<rclcpp::Node>(
        node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // 创建 Move Group 接口
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = MoveGroupInterface(node2, control_group);

    // 声明并运行 Action Server
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
