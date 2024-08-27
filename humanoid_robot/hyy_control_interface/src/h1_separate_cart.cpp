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

void handle_sigint(int sig) {
    printf("\nros2 is shutting down, robot stop moving.\n");
    stop.store(true);
    rightArmControl->stopRun();
    leftArmControl->stopRun();
    bodyControl->stopRun();
    headControl->stopRun();
    printf("ros2 shut down, wait seconds to exit 0.\n");
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
    /*   Standard application structure                                        */
    /*   System Initialize!!                                                   */
    /*                                                                         */
    /***************************************************************************/

    rclcpp::init(argc, argv);
    signal(SIGINT, handle_sigint);
    auto node = rclcpp::Node::make_shared("hyyShowDemo");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
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
    /*   Standard application structure                                        */
    /*   Get Variables!!                                                       */
    /*                                                                         */
    /***************************************************************************/

    string R0_VEL = "R0_PERCENT5";
    string R1_VEL = "R0_PERCENT5";
    string A0_VEL = "A0_PERCENT10";
    string A1_VEL = "A0_PERCENT10";

    // string zone = "DEFAULT_ZONE";
    // string tool = "DEFAULT_TOOL";
    // string wobj = "DEFAULT_WOBJ";

    leftArmControl->isblock(true);
    rightArmControl->isblock(true);
    bodyControl->isblock(true);
    headControl->isblock(true);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Left Arm Move Test.                                                   */
    /*                                                                         */
    /***************************************************************************/

    int count = 0;
    while (leftArmControl->robot_ok())
    {
        //  STEP 1 (Initialize)
        leftArmControl->moveA("R0_J0", R0_VEL);
        //  STEP 2 (Grip)
        leftArmControl->moveJ("R0_P2", R0_VEL);
        //  STEP 4 (Back to Initial)
        leftArmControl->moveJ("R0_P0", R0_VEL);

        if (++count >= 1){
            break;
        }
    }

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Right Arm Move Test.                                                  */
    /*                                                                         */
    /***************************************************************************/

    count = 0;
    while (rightArmControl->robot_ok())
    {
        //  STEP 1 (Initialize)
        rightArmControl->moveA("R1_J0", R1_VEL);

        //  STEP 2 (Grip)
        rightArmControl->moveJ("R1_P2", R1_VEL);

        //  STEP 4 (Back to Initial)
        rightArmControl->moveJ("R1_P0", R1_VEL);
        
        if (++count >= 1){
            break;
        }
    }

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Body Move Test.                                                       */
    /*                                                                         */
    /***************************************************************************/

    count = 0;
    while (bodyControl->robot_ok())
    {
        //  STEP 1 (Initialize)
        bodyControl->moveA("A0_J0", A0_VEL);    

        //  STEP 2 (Grip)
        bodyControl->moveA("A0_J1", A0_VEL);

        //  STEP 3 (Move and Drop)
        bodyControl->moveA("A0_J0", A0_VEL);    
        bodyControl->moveA("A0_J2", A0_VEL);
        bodyControl->moveA("A0_J3", A0_VEL);

        //  STEP 4 (Back to Initial)
        bodyControl->moveA("A0_J2", A0_VEL);
        bodyControl->moveA("A0_J0", A0_VEL);

        if (++count >= 1){
            break;
        }
    }
    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Head Move Test.                                                       */
    /*                                                                         */
    /***************************************************************************/

    count = 0;
    while (headControl->robot_ok())
    {
        //  STEP 1 (Initialize)
        headControl->moveA("A1_J0", A1_VEL);

        //  STEP 3 (Move and Drop)
        headControl->moveA("A1_J1", A1_VEL);
        headControl->moveA("A1_J0", A1_VEL);

        if (++count >= 1){
            break;
        }
    }

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   System Exit.                                                          */
    /*                                                                         */
    /***************************************************************************/

    RCLCPP_INFO(node->get_logger(), "debug finished test");
    executor->cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}