#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hyy_message/action/move_xyzw.hpp"
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// Global variables for control group and base frame, and the MoveGroupInterface instance
std::string control_group = "none";
std::string base_frame = "none";
moveit::planning_interface::MoveGroupInterface move_group_interface;

namespace
{
constexpr double kPlanningTimeSec = 5.0;
constexpr unsigned int kPlanningAttempts = 10;
constexpr double kGoalPositionTolerance = 0.005;      // 5 mm
constexpr double kGoalOrientationTolerance = 0.0873;  // 5 deg
constexpr double kGoalJointTolerance = 0.01;
constexpr double kCartesianDistanceThreshold = 0.45;  // m
constexpr double kCartesianOrientationThreshold = 0.0873;  // 5 deg
constexpr double kCartesianEEFStep = 0.005;  // m
constexpr double kCartesianJumpThreshold = 1.5;
constexpr double kCartesianMinFraction = 0.999;

double poseDistance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)
{
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationDistance(const geometry_msgs::msg::Quaternion& a, const geometry_msgs::msg::Quaternion& b)
{
    tf2::Quaternion qa;
    tf2::Quaternion qb;
    tf2::fromMsg(a, qa);
    tf2::fromMsg(b, qb);
    double dot = std::abs(qa.dot(qb));
    if (dot > 1.0) {
        dot = 1.0;
    }
    return 2.0 * std::acos(dot);
}
}

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

            auto current_pose = move_group_interface.getCurrentPose();

            // Set up the reference frame, current state, speed/acceleration
            move_group_interface.setPoseReferenceFrame(base_frame);
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPlanningPipelineId("ompl");
            move_group_interface.setPlannerId("RRTConnectkConfigDefault");
            move_group_interface.setPlanningTime(kPlanningTimeSec);
            move_group_interface.setNumPlanningAttempts(kPlanningAttempts);
            move_group_interface.allowReplanning(true);
            move_group_interface.setGoalPositionTolerance(kGoalPositionTolerance);
            move_group_interface.setGoalOrientationTolerance(kGoalOrientationTolerance);
            move_group_interface.setGoalJointTolerance(kGoalJointTolerance);
            move_group_interface.setMaxVelocityScalingFactor(goal->speed);
            move_group_interface.setMaxAccelerationScalingFactor(goal->accel);

            RCLCPP_INFO(
                this->get_logger(),
                "Move group=%s, planner=%s, eef=%s, frame=%s",
                control_group.c_str(),
                move_group_interface.getPlannerId().c_str(),
                move_group_interface.getEndEffectorLink().c_str(),
                move_group_interface.getPlanningFrame().c_str());
            RCLCPP_INFO(
                this->get_logger(),
                "Current pose -> x: %.3f, y: %.3f, z: %.3f",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);

            const double linear_distance = poseDistance(current_pose.pose, pose);
            const double angular_distance =
                orientationDistance(current_pose.pose.orientation, pose.orientation);

            moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
            bool planned = false;
            bool success = false;
            std::string planning_mode = "unset";

            // Nearby same-orientation moves are much more stable as Cartesian paths
            // and avoid jumping to a different IK branch.
            if (linear_distance <= kCartesianDistanceThreshold &&
                angular_distance <= kCartesianOrientationThreshold) {
                moveit_msgs::msg::RobotTrajectory cartesian_msg;
                std::vector<geometry_msgs::msg::Pose> waypoints{pose};
                const double fraction = move_group_interface.computeCartesianPath(
                    waypoints, kCartesianEEFStep, kCartesianJumpThreshold, cartesian_msg, true);

                RCLCPP_INFO(
                    this->get_logger(),
                    "%s - MoveXYZW: cartesian trial distance=%.3f m angle=%.3f rad fraction=%.3f",
                    control_group.c_str(),
                    linear_distance,
                    angular_distance,
                    fraction);

                if (fraction >= kCartesianMinFraction) {
                    auto current_state = move_group_interface.getCurrentState();
                    if (current_state) {
                        robot_trajectory::RobotTrajectory cartesian_traj(
                            move_group_interface.getRobotModel(), control_group);
                        cartesian_traj.setRobotTrajectoryMsg(*current_state, cartesian_msg);

                        trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
                        if (time_param.computeTimeStamps(cartesian_traj, goal->speed, goal->accel)) {
                            cartesian_traj.getRobotTrajectoryMsg(cartesian_msg);
                            planning_mode = "cartesian";
                            planned = true;
                            success = (move_group_interface.execute(cartesian_msg) ==
                                       moveit::core::MoveItErrorCode::SUCCESS);
                        } else {
                            RCLCPP_WARN(
                                this->get_logger(),
                                "%s - MoveXYZW: cartesian time parameterization failed, fallback to OMPL.",
                                control_group.c_str());
                        }
                    } else {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "%s - MoveXYZW: failed to query current state for cartesian execution, fallback to OMPL.",
                            control_group.c_str());
                    }
                }
            }

            // Prefer planning to a concrete IK solution first. This is usually
            // more stable than directly sampling pose constraints near the
            // workspace boundary.
            if (!planned) {
                move_group_interface.clearPoseTargets();
                if (move_group_interface.setJointValueTarget(pose)) {
                    planned = (move_group_interface.plan(ompl_plan) ==
                               moveit::core::MoveItErrorCode::SUCCESS);
                    planning_mode = "joint_ik";
                } else {
                    RCLCPP_WARN(this->get_logger(), "%s - MoveXYZW: exact IK target setup failed.", control_group.c_str());
                }
            }

            if (!planned) {
                move_group_interface.clearPoseTargets();
                move_group_interface.setStartStateToCurrentState();
                move_group_interface.setPoseTarget(pose);
                planned = (move_group_interface.plan(ompl_plan) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
                planning_mode = "pose_target";
            }

            if (!planned) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s - MoveXYZW: planning failed for target x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
                    control_group.c_str(),
                    goal->positionx,
                    goal->positiony,
                    goal->positionz,
                    goal->roll,
                    goal->pitch,
                    goal->yaw);
                result_->result = "MoveXYZW:FAILED";
                goal_handle->abort(result_);
                move_group_interface.clearPoseTargets();
                return;
            }

            if (planning_mode != "cartesian") {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s - MoveXYZW: planning succeeded via %s in %.3f s",
                    control_group.c_str(),
                    planning_mode.c_str(),
                    ompl_plan.planning_time_);

                success = (move_group_interface.execute(ompl_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                move_group_interface.clearPoseTargets();
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%s - MoveXYZW: cartesian execution selected for this move.",
                    control_group.c_str());
            }

            // Report success or failure of the action
            if (success) {
                RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: execution successful via %s!", control_group.c_str(), planning_mode.c_str());
                result_->result = "MoveXYZW:SUCCESS";
                goal_handle->succeed(result_);
            } else {
                RCLCPP_WARN(this->get_logger(), "%s - MoveXYZW: execution failed via %s!", control_group.c_str(), planning_mode.c_str());
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
