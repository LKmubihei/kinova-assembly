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

    gripperHandControl->Gripper_initialize();
    
    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Get Variables!!                      */
    /*                                                                         */
    /***************************************************************************/
	
    string R0_VEL = "R0_PERCENT10";
    string R0_CATCH_VEL = "R0_PERCENT5";
    string R1_VEL = "R0_PERCENT10";
    string R1_TWIST_VEL = "R0_PERCENT30";
    string R0_VEL_SHAKE = "R0_PERCENT10";
    string A0_VEL = "A0_PERCENT01";
    string A1_VEL = "A1_PERCENT1";

    vector<int> force_hand = {800, 800, 800, 800, 800, 800};
    vector<int> angle_hand_entry = {1000, 1000, 1000, 1000, 1000, 5};
    vector<int> angle_hand_catch = {400, 400, 400, 400, 600, 5};
    int angle_gripper = 100;
    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Robot Movement!!                     */
    /*                                                                         */
    /***************************************************************************/

    rightArmControl->isblock(false);
    leftArmControl->isblock(false);
    bodyControl->isblock(false);
    headControl->isblock(false);

    //  STEP 1 (INITIAL)
    gripperHandControl->hand_SetForce(force_hand);
    gripperHandControl->Gripper_fullopen();
    gripperHandControl->hand_fullopen();
    rightArmControl->moveA("R0_T1_INITIAL", R0_VEL);
    leftArmControl->moveA("R1_T1_INITIAL", R1_VEL);
    bodyControl->moveA("A0_T1_INITIAL", A0_VEL);    
    headControl->moveA("A1_T1_INITIAL", A1_VEL);
    blockhere(4, leftArmControl->ask_status(), rightArmControl->ask_status(), bodyControl->ask_status(), headControl->ask_status());

    //  STEP 2 (GOTO BOTTLE)
    rightArmControl->isblock(true);
    rightArmControl->moveA("R0_T1_CATCH_ENTRY", R0_VEL);
    gripperHandControl->hand_SetAngle(angle_hand_entry);
    rightArmControl->moveA("R0_T1_CATCH", R0_CATCH_VEL);

    //  STEP 3 (CATCH BOTTLE)
    usleep(500000);
    gripperHandControl->hand_SetAngle(angle_hand_catch);
    usleep(500000);

    //  STEP 4 (LIFT BOTTLE)
    rightArmControl->moveA("R0_T1_LIFT", R0_CATCH_VEL);
    usleep(500000);
    
    //  STEP 5 (TAKE BACK BOTTLE)
    rightArmControl->moveA("R0_T1_CATCH_ENTRY", R0_VEL);
    
    //  STEP 6 (HOLD BOTTLE AND TWIST BOTTLE CAP)
    rightArmControl->isblock(false);
    rightArmControl->moveA("R0_T1_HOLD", R0_VEL);
    leftArmControl->isblock(true);
    leftArmControl->moveA("R1_T1_TWIST_ENTRY", R1_VEL);
    leftArmControl->moveA("R1_T1_TWIST", R1_VEL);
    usleep(500000);
    // sleep(500);
    gripperHandControl->Gripper_goto(angle_gripper);
    usleep(500000);
    leftArmControl->moveA("R1_T1_TWIST_END", R1_TWIST_VEL);
    leftArmControl->moveA("R1_T1_TWIST_END_BACK", R1_VEL);

    // STEP 7 (HOLD BOTTLE CAP AND PLACE BOTTLE)
    leftArmControl->isblock(false);
    leftArmControl->moveA("R1_T1_HOLD_CAP", R1_VEL);
    rightArmControl->isblock(true);
    rightArmControl->moveA("R0_T1_CATCH_ENTRY", R0_VEL);
    rightArmControl->moveA("R0_T1_LIFT", R0_VEL);
    usleep(250000);
    rightArmControl->moveA("R0_T1_PLACE", R0_CATCH_VEL);
    usleep(500000);
    gripperHandControl->hand_SetAngle(angle_hand_entry);
    usleep(500000);
    rightArmControl->moveA("R0_T1_CATCH_ENTRY", R0_CATCH_VEL);
    
    // STEP 7 (BACK TO INTIAL POSITION)
    rightArmControl->isblock(false);
    rightArmControl->moveA("R0_T1_INITIAL", R0_VEL);
    leftArmControl->moveA("R1_T1_INITIAL", R1_VEL);
    gripperHandControl->Gripper_fullopen();
    gripperHandControl->hand_fullopen();
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