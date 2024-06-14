#include "hyy_robot_control/hyy_robot_control.h"

using namespace std;

int main(int argc, char **argv){
    
    
    //--------------------------------------------intialize---------------------------------------------//
    

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hyyRobotControl");
    hyy_robot_control::HyyRobotControl hyyGripperControl(node);
    if (!hyyGripperControl.init("hyy_external_device_controller", 1)){
        RCLCPP_ERROR(node->get_logger(), "hyy_robot_control client initialize falied");
        return -1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::thread spinner([executor, node]()
        { 
            executor->add_node(node);
            executor->spin(); 
        }
    );

    hyyGripperControl.Gripper_initialize();
    RCLCPP_INFO(node->get_logger(), "Gripper pos: %d", hyyGripperControl.Gripper_GetPos());
    sleep(1);
    hyyGripperControl.Gripper_fullclose();
    RCLCPP_INFO(node->get_logger(), "Gripper pos: %d", hyyGripperControl.Gripper_GetPos());
    sleep(1);
    hyyGripperControl.Gripper_fullopen();
    RCLCPP_INFO(node->get_logger(), "Gripper pos: %d", hyyGripperControl.Gripper_GetPos());
    sleep(1);
    hyyGripperControl.Gripper_fullclose();
    RCLCPP_INFO(node->get_logger(), "Gripper pos: %d", hyyGripperControl.Gripper_GetPos());
    sleep(1);
    hyyGripperControl.Gripper_fullopen();
    RCLCPP_INFO(node->get_logger(), "Gripper pos: %d", hyyGripperControl.Gripper_GetPos());

    //-----------------------------------------fancy movement end---------------------------------------//

    RCLCPP_INFO(node->get_logger(), "Gripper finished test");
    executor->cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}