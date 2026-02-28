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
#include "hyy_message/action/move_xyzw.hpp"
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// Global variables for control group and base frame, and the MoveGroupInterface instance
std::string control_group = "none";
std::string base_frame = "none";
moveit::planning_interface::MoveGroupInterface move_group_interface;

class ParamServer : public rclcpp::Node
{
    public:
        // Constructor for the parameter server node
        ParamServer() : Node("param_server") 
        {
            // Declare and retrieve 'control_group' parameter
            this->declare_parameter("control_group", "null");
            control_group = this->get_parameter("control_group").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "control_group received -> %s", control_group.c_str());

            // Declare and retrieve 'base_frame' parameter
            this->declare_parameter("base_frame", "null");
            base_frame = this->get_parameter("base_frame").get_parameter_value().get<std::string>();
            RCLCPP_INFO(this->get_logger(), "base_frame received -> %s", base_frame.c_str());
        }
    private:
};

class ActionServer : public rclcpp::Node
{
    public:
        // Define action types using the custom MoveXYZW action
        using MoveXYZW = hyy_message::action::MoveXYZW;
        using GoalHandle = rclcpp_action::ServerGoalHandle<MoveXYZW>;

        // Constructor for the action server node
        explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node(control_group + "_ActionServer", options)
        {
            // Create the action server for the MoveXYZW action
            action_server_ = rclcpp_action::create_server<MoveXYZW>(
                this,
                "/MoveXYZW",
                std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
        }

    private:
        rclcpp_action::Server<MoveXYZW>::SharedPtr action_server_;

        // Callback to handle a received goal request
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveXYZW::Goal> goal)
        {
            // Log the received goal parameters (position and orientation)
            RCLCPP_INFO(this->get_logger(), "Received POSE GOAL -> x: %.2f, y: %.2f, z: %.2f", goal->positionx, goal->positiony, goal->positionz);
            RCLCPP_INFO(this->get_logger(), "Orientation (RPY) -> yaw: %.2f, pitch: %.2f, roll: %.2f", goal->yaw, goal->pitch, goal->roll);
            
            // Accept and execute the received goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Callback to handle accepted goals
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            // Start a new thread to handle the goal execution to avoid blocking
            std::thread(
                [this, goal_handle]() {
                    execute(goal_handle);
                }).detach();
        }

        // Callback to handle a goal cancellation request
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            // Log the cancellation request and stop the current motion
            RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
            move_group_interface.stop(); // Stop the robot motion

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Execute the goal: move to the desired waypoint using MoveIt!2
        void execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Starting MoveXYZW motion to desired waypoint...");
            auto result_ = std::make_shared<MoveXYZW::Result>();

            // Create a vector of waypoints for the robot to follow
            std::vector<geometry_msgs::msg::Pose> waypoints;

            // Get the goal and convert it into a Pose message
            const auto goal = goal_handle->get_goal();
            geometry_msgs::msg::Pose pose;
            pose.position.x = goal->positionx;
            pose.position.y = goal->positiony;
            pose.position.z = goal->positionz;

            // Convert roll, pitch, yaw to a quaternion for orientation
            tf2::Quaternion q;
            q.setRPY(goal->roll, goal->pitch, goal->yaw);
            pose.orientation = tf2::toMsg(q);

            waypoints.push_back(pose);

            // Set up the reference frame, current state, and speed/acceleration
            auto speed_ = goal->speed;
            auto accel_ = goal->accel;
            move_group_interface.setPoseReferenceFrame(base_frame); 
            move_group_interface.setStartStateToCurrentState();
            
            // Plan the Cartesian path with waypoints
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
            
            // Check if the path planning was successful
            if (fraction < 0.99) {
                RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the planned path has been completed, planning may fail.", fraction * 100.0);
            }

            // Use time parameterization to adjust the trajectory for speed and acceleration
            robot_trajectory::RobotTrajectory rt(move_group_interface.getRobotModel(), move_group_interface.getName());
            rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(rt, speed_, accel_);
            rt.getRobotTrajectoryMsg(trajectory);

            // Execute the planned trajectory
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;

            bool success = (move_group_interface.execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            // Report success or failure of the action
            if (success) {
                RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: Cartesian path execution successful!", control_group.c_str());
                result_->result = "MoveXYZW:SUCCESS";
                goal_handle->succeed(result_);
            } else {
                RCLCPP_WARN(this->get_logger(), "%s - MoveXYZW: Cartesian path execution failed!", control_group.c_str());
                result_->result = "MoveXYZW:FAILED";
                goal_handle->abort(result_);
            } 
        }
};

int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the parameter server node and get the required parameters
    auto node_PARAM = std::make_shared<ParamServer>();
    rclcpp::spin_some(node_PARAM);

    // Create and run the MoveIt!2 interface node
    auto name = "_MoveXYZW_interface";
    auto node2name = control_group + name;
    auto const node2 = std::make_shared<rclcpp::Node>(
        node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    // Use a single-threaded executor to spin the MoveIt!2 node
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Initialize the MoveGroupInterface for motion planning
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = MoveGroupInterface(node2, control_group);

    // Create and start the action server
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
