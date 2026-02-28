#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_moveit");

// callback functions of GripperCommand action
void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle);
void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback);
void result_callback(const GoalHandleGripperCommand::WrappedResult & result);

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_moveit");

    // action client for GripperCommand action
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client = 
        rclcpp_action::create_client<GripperCommand>(node, "robotiq_gripper_controller/gripper_cmd");

    // get desired gripper position from parameter server or command line
    node->declare_parameter("gripper_position", 0.0);
    double gripper_position = node->get_parameter("gripper_position").as_double();
    // it must be in the scope [0, 0.8)
    gripper_position = [gripper_position](){
        if(gripper_position < 0){
            return 0.0;
        }else if(gripper_position > 0.8){
            return 0.8;
        }else{
            return gripper_position;
        }
    }();

    // We spin up a SingleThreadedExecutor to handle any callbacks that may happen
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // wait for action server to start
    RCLCPP_INFO(LOGGER, "Waiting for action server");
    if(!gripper_client->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    // action message for GripperCommand action
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = gripper_position;
    goal_msg.command.max_effort = -1.0;
    RCLCPP_INFO(LOGGER, "Sending goal");
    // set callback functions
    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&goal_response_callback, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&result_callback, std::placeholders::_1);
    // send goal
    auto result = gripper_client->async_send_goal(goal_msg, send_goal_options);

    // wait for the result
    if(result.wait_for(std::chrono::seconds(10)) == std::future_status::ready){
        RCLCPP_INFO(LOGGER, "Result received");
    }else{
        RCLCPP_ERROR(LOGGER, "Result not received after 10 seconds");
    }



    rclcpp::shutdown();
    return 0;
}

/**
 * @brief callback function for GripperCommand action goal response
 * 
 * @param goal_handle 
 */
void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
  } else {
    RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
  }
}

/**
 * @brief callback function for GripperCommand action feedback
 * 
 * @param feedback 
 */
void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(LOGGER, "Got Feedback: Current position is %f", feedback->position);
}

/**
 * @brief callback function for GripperCommand action result
 * 
 * @param result 
 */
void result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(LOGGER, "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(LOGGER, "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(LOGGER, "Unknown result code");
    return;
  }
  RCLCPP_INFO(LOGGER, "Goal is completed, current position is %f", result.result->position);
}