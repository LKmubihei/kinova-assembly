#include <stdexcept>
#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include "hyy_message/srv/setangle.hpp" // 根据你的服务文件位置修改包含路径

class RobotiqGripperNode : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;
  using Setangle = hyy_message::srv::Setangle;

  RobotiqGripperNode(const std::string & node_name = "gripper_cmd_Node",
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node(node_name, options)
  {
    gripper_action_name_ = "/robotiq_gripper_controller/gripper_cmd";

    // 创建Action Client
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);
    send_goal_options_.goal_response_callback = std::bind(&RobotiqGripperNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&RobotiqGripperNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&RobotiqGripperNode::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Waiting for action server to become available...");
    if(!gripper_action_client_->wait_for_action_server(std::chrono::seconds(10))){
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      throw std::runtime_error("Action server not available");
    }
    RCLCPP_INFO(this->get_logger(), "Action server ready.");

    // 创建Service服务端
    gripper_service_ = this->create_service<Setangle>(
      "~/ur_gripper_cmd_server",
      std::bind(&RobotiqGripperNode::controlgripper_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Gripper command service is ready.");
  }

  void grasp(double gripper_position) {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = gripper_position;
    goal_msg.command.max_effort = -1.0; // 不限制力度

    RCLCPP_INFO(this->get_logger(), "Sending gripper goal with position: %f", gripper_position);
    auto future_goal_handle = gripper_action_client_->async_send_goal(goal_msg, send_goal_options_);
  }

private:
  void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
    if(!goal_handle){
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
  }

  void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                         const std::shared_ptr<const GripperCommand::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Got Feedback: Current position: %f", feedback->position);
  }

  void result_callback(const GoalHandleGripperCommand::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Gripper action succeeded, final position: %f", result.result->position);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  void controlgripper_callback(
    const std::shared_ptr<Setangle::Request> request,
    const std::shared_ptr<Setangle::Response> response)
  {
    // 从request中读angle，这里假定angle[1]是我们要发送给gripper的位置
    double value = -0.0007 * request->angle[1] + 0.7;
    RCLCPP_INFO(this->get_logger(), "Received request: status='%s', pos=%f", request->status.c_str(), value);

    // 调用grasp函数将目标位置发送给gripper action服务器
    this->grasp(value);

    response->angle_accepted = true;
  }

  std::string gripper_action_name_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp_action::Client<GripperCommand>::SendGoalOptions send_goal_options_;
  rclcpp::Service<Setangle>::SharedPtr gripper_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotiqGripperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
