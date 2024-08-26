#include "hyy_control_interface/hyy_control_interface.h"
#include <atomic>
#include <csignal>
#include <cstdarg>
#include <iostream>
#include <thread>

using namespace std;

extern std::atomic<bool> stop;
std::shared_ptr<hyy_control_interface::HyyRobotControl> rightArmControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> leftArmControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> bodyControl;
std::shared_ptr<hyy_control_interface::HyyRobotControl> headControl;
// std::shared_ptr<hyy_control_interface::HyyExternalDevicesControl> gripperHandControl;

void handle_sigint(int sig) {
    printf("\nros2 is shutting down, robot stop moving.\n");
    stop.store(true);
    // rightArmControl->stopDeviceRun();
    rightArmControl->stopRobotRun();
    leftArmControl->stopRobotRun();
    bodyControl->stopAddaxisRun();
    headControl->stopAddaxisRun();
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

    // gripperHandControl = std::make_shared<hyy_control_interface::HyyExternalDevicesControl>(node);
    // if (!gripperHandControl->init("hyy_external_device_controller")){
    //     RCLCPP_ERROR(node->get_logger(), "gripperHandControl client initialize falied");
    //     return -1;
    // }
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

    // gripperHandControl->Gripper_initialize();
    
    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Get Variables!!                      */
    /*                                                                         */
    /***************************************************************************/
	
    string R0_VEL = "R0_PERCENT20";
    string R1_VEL = "R0_PERCENT20";
    string R0_VEL_SHAKE = "R0_PERCENT30";
    string A0_VEL = "A0_PERCENT01";
    string A1_VEL = "A1_PERCENT1";

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Robot Movement!!                     */
    /*                                                                         */
    /***************************************************************************/

    rightArmControl->isblock(false);
    leftArmControl->isblock(false);
    bodyControl->isblock(false);
    headControl->isblock(false);

    //  STEP 1 (RESET)
    rightArmControl->moveA("R0_T1_INITIAL", R0_VEL);
    leftArmControl->moveA("R1_T1_INITIAL", R1_VEL);
    bodyControl->moveA("A0_T1_INITIAL", A0_VEL);    
    headControl->moveA("A1_T1_INITIAL", A1_VEL);
    blockhere(4, leftArmControl->ask_status(), rightArmControl->ask_status(), bodyControl->ask_status(), headControl->ask_status());

    //  STEP 2 (INITIAL POSITION)
    bodyControl->moveA("A0_T1_SHAKE", A0_VEL);    
    headControl->moveA("A1_T1_SHAKE", A1_VEL);
    blockhere(2, bodyControl->ask_status(), headControl->ask_status());

    rightArmControl->isblock(true);
    rightArmControl->moveA("R0_T1_SHAKE_INITIAL", R0_VEL);

    //  STEP 4 (SHAKE HAND LOOP)
    int count = 0;
    while (rightArmControl->robot_ok())
    {
        usleep(250000);
        rightArmControl->moveA("R0_T1_SHAKE_FINAL", R0_VEL_SHAKE);
        usleep(250000);
        rightArmControl->moveA("R0_T1_SHAKE_INITIAL", R0_VEL_SHAKE);
        if (count++ >= 2){
            break;
        }
    }

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