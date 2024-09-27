#include "hyy_control_interface/hyy_control_interface.h"
#include <atomic>
#include <csignal>
#include <cstdarg>
#include <iostream>
#include <thread>
#include <vector>

using namespace std;

extern std::atomic<bool> stop;
std::shared_ptr<hyy_control_interface::HyyRobotControl> rightArmControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> leftArmControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> bodyControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> headControl;
std::shared_ptr<hyy_control_interface::HyyExternalDevicesControl> gripperHandControl;

void handle_sigint(int sig) {
    printf("\nros2 is shutting down, robot stop moving.\n");
    stop.store(true);
    rightArmControl->stopRun();
    leftArmControl->stopRun();
    bodyControl->stopRun();
    headControl->stopRun();
    printf("\nros2 shut down, wait seconds to exit 0.\n");
    if (rclcpp::shutdown())
    {
        sleep(2);
        exit(0);
    }
}

void blockhere(int num_args, ...) {
    va_list args;
    va_start(args, num_args);

    while (1) {
        if (stop.load()) {
            break;
        }

        bool all_zero = true;

        va_list args_copy;
        va_copy(args_copy, args);

        for (int i = 0; i < num_args; ++i) {
            if (va_arg(args_copy, int) != 0) {
                all_zero = false;
                break;
            }
        }

        va_end(args_copy);

        if (all_zero) {
            break;
        } else {
            usleep(100000);
        }
    }

    va_end(args);
}

int main(int argc, char **argv){
    
    
    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Client Initialize!!                  */
    /*                                                                         */
    /***************************************************************************/

    rclcpp::init(argc, argv);
    signal(SIGINT, handle_sigint);
    auto node = rclcpp::Node::make_shared("hyyShowDemo");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    gripperHandControl = std::make_shared<hyy_control_interface::HyyExternalDevicesControl>(node);
    if (!gripperHandControl->init("hyy_external_device_controller")){
        RCLCPP_ERROR(node->get_logger(), "gripperHandControl client initialize falied");
        return -1;
    }
    leftArmControl = std::make_shared<hyy_control_interface::HyyRobotControl>(node);
    if (!leftArmControl->init("hyy_left_arm_controller")){
        RCLCPP_ERROR(node->get_logger(), "leftArmControl client initialize falied");
        return -1;
    }
    rightArmControl = std::make_shared<hyy_control_interface::HyyRobotControl>(node);
    if (!rightArmControl->init("hyy_right_arm_controller")){
        RCLCPP_ERROR(node->get_logger(), "rightArmControl client initialize falied");
        return -1;
    }
    bodyControl = std::make_shared<hyy_control_interface::HyyRobotControl>(node);
    if (!bodyControl->init("hyy_body_controller")){
        RCLCPP_ERROR(node->get_logger(), "bodyControl initialize falied");
        return -1;
    }
    headControl = std::make_shared<hyy_control_interface::HyyRobotControl>(node);
    if (!headControl->init("hyy_head_controller")){
        RCLCPP_ERROR(node->get_logger(), "headControl client initialize falied");
        return -1;
    }
    executor->add_node(node);
    std::thread spinner([executor]()
        { 
            executor->spin(); 
        }
    );
    sleep(2);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Ros2 code !!                         */
    /*                                                                         */
    /***************************************************************************/

    // gripperHandControl->Gripper_initialize();
    gripperHandControl->Gripper_fullopen();
    gripperHandControl->hand_fullopen();
    gripperHandControl->Gripper_SetForce(150);
    
    vector<int> speed_hand = {800, 800, 800, 800, 800, 800};
    vector<int> force_hand = {300, 300, 300, 300, 300, 300};
    gripperHandControl->hand_SetSpeed(speed_hand);
    gripperHandControl->hand_SetForce(speed_hand);
    vector<int> angle_hand_1 = {1000, 1000, 1000, 1000, 1000, 5};
    vector<int> angle_hand_2 = {450, 450, 450, 450, 620, 5};
    int angle_gripper = 105;

    gripperHandControl->hand_SetAngle(angle_hand_1);
    sleep(10);
    gripperHandControl->hand_SetAngle(angle_hand_2);
    gripperHandControl->Gripper_goto(angle_gripper);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Ros2 destroy!!                       */
    /*                                                                         */
    /***************************************************************************/

    RCLCPP_INFO(node->get_logger(), "debug finished test");
    executor->cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}