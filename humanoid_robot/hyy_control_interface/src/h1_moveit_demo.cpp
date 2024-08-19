#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_plan_demo");

int main(int argc, char** argv)
{

  /***************************************************************************/
  /*                                                                         */
  /*   Standard application structure   Moveit Initize!!                     */
  /*                                                                         */
  /***************************************************************************/

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("motion_plan_node", node_options);

  move_group_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  /***************************************************************************/
  /*                                                                         */
  /*   Standard application structure   Moveit Plan !!                       */
  /*                                                                         */
  /***************************************************************************/

  // set MoveIt planning groups 
  static const std::string PLANNING_GROUP = "left_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Visualization
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "motion_plan", move_group.getRobotModel());

  // Remote control allows users to step through a high level script via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // plan a trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Planning to a Pose goal
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 0.707;
  target_pose1.orientation.x = 0;
  target_pose1.orientation.y = 0.707;
  target_pose1.orientation.z = 0;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
 
  /***************************************************************************/
  /*                                                                         */
  /*   Standard application structure   Moveit execute !!                    */
  /*                                                                         */
  /***************************************************************************/

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move");

  // Moving to a pose goal
  move_group.move();
 
  /***************************************************************************/
  /*                                                                         */
  /*   Standard application structure   Moveit destroy!!                     */
  /*                                                                         */
  /***************************************************************************/

  rclcpp::shutdown();
  return 0;
}
