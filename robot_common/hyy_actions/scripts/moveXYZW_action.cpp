#include <functional>
#include <memory>
#include <thread>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "hyy_message/action/move_xyzw.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

// Declaration of GLOBAL VARIABLE --> ROBOT / END-EFFECTOR PARAMETER:
std::string control_group = "none";
std::string base_frame = "none";

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

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

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
    
    // Function that checks the goal received, and accepts it accordingly:
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
        //(void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal received.
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        move_group_interface.stop();

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        RCLCPP_INFO(this->get_logger(), "Starting MoveXYZW motion to desired waypoint...");
        auto result = std::make_shared<MoveXYZW::Result>();
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(control_group);
        
        /***************************************************************************/
        /*                                                                         */
        /*   Standard application structure                                        */
        /*                                                                         */
        /***************************************************************************/

        // Joint model group:
        
        // tf2_ros::Buffer tf_buffer(this->get_clock());
        // tf2_ros::TransformListener tf_listener(tf_buffer);
        // geometry_msgs::msg::TransformStamped transform_stamped;
        // if (tf_buffer.canTransform("t1_base", "base_link", tf2::TimePointZero, std::chrono::seconds(1))) {
        //     try {
        //         transform_stamped = tf_buffer.lookupTransform("t1_base", "base_link", tf2::TimePointZero);
        //     } catch (tf2::TransformException &ex) {
        //         RCLCPP_ERROR(this->get_logger(), "Could not transform base_link to t1_base: %s", ex.what());
        //         return;
        //     }
        // }else{
        //     RCLCPP_ERROR(this->get_logger(), "Transform from base_link to t1_base is not available within the given time frame.");
        // }

        // // Get CURRENT POSE:
        // auto current_pose = move_group_interface.getCurrentPose();
        // // Transform the current pose to the new reference frame
        // geometry_msgs::msg::PoseStamped transformed_pose;
        // tf2::doTransform(current_pose, transformed_pose, transform_stamped);

        // RCLCPP_INFO(this->get_logger(), "Current POSE before the new MoveXYZW was:");
        // RCLCPP_INFO(this->get_logger(), "POSITION -> (x = %.2f, y = %.2f, z = %.2f)", current_pose.pose.position.x, current_pose.pose.position.y,current_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);
        // RCLCPP_INFO(this->get_logger(), "Transformed POSITION -> (x = %.2f, y = %.2f, z = %.2f)", transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "Transformed ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);

        /***************************************************************************/
        /*                                                                         */
        /*   Standard application structure                                        */
        /*                                                                         */
        /***************************************************************************/

        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Pose pose;
        pose.position.x = goal->positionx;
        pose.position.y = goal->positiony;
        pose.position.z = goal->positionz;

        tf2::Quaternion q;
        q.setRPY(goal->roll, goal->pitch, goal->yaw);
        pose.orientation = tf2::toMsg(q);

        /***************************************************************************/
        /*                                                                         */
        /*   Standard application structure                                        */
        /*                                                                         */
        /***************************************************************************/

        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        // Set the tolerance for reaching the position target (m/radian)
        move_group_interface.setGoalPositionTolerance(0.005);   
        move_group_interface.setGoalOrientationTolerance(0.005); 
        // Set the maximum planning time (s)
        move_group_interface.setPlanningTime(10.0);
        move_group_interface.setPoseReferenceFrame(base_frame); 
        move_group_interface.setPoseTarget(pose);

        // Plan, execute and inform (with feedback):
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success) {

            RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: Planning successful!", control_group.c_str());
            move_group_interface.move();
            
            // Do if GOAL CANCELLED:
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = "MoveXYZW:CANCELED";
                goal_handle->canceled(result);
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: Movement executed!", control_group.c_str());
                result->result = "MoveXYZW:SUCCESS";
                goal_handle->succeed(result);
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "%s - MoveXYZW: Planning failed!", control_group.c_str());
            result->result = "MoveXYZW:FAILED";
            goal_handle->succeed(result);
        }    

    }

};

int main(int argc, char ** argv)
{
  // Initialise MAIN NODE:
  rclcpp::init(argc, argv);

  // Obtain ParamServer parameter:
  auto node_PARAM = std::make_shared<ParamServer>();
  rclcpp::spin_some(node_PARAM);

  // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
  auto name = "_MoveXYZW_interface";
  auto node2name = control_group + name;
  auto const node2 = std::make_shared<rclcpp::Node>(
      node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the Move Group Interface:
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, control_group);
  // Create the MoveIt PlanningScene Interface:
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Declare and spin ACTION SERVER:
  auto action_server = std::make_shared<ActionServer>();
  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}