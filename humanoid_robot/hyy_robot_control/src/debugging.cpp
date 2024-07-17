#include "hyy_robot_control/hyy_robot_control.h"
#include <atomic>
#include <csignal>
#include <cstdarg>
#include <iostream>
#include <thread>

using namespace std;

extern std::atomic<bool> stop;
std::shared_ptr<hyy_robot_control::HyyRobotControl> hyyRobotRightArmControl;
std::shared_ptr<hyy_robot_control::HyyRobotControl> hyyRobotLeftArmControl;
std::shared_ptr<hyy_robot_control::HyyRobotControl> hyyRobotBodyControl;
std::shared_ptr<hyy_robot_control::HyyRobotControl> hyyRobotHeadControl;
// std::shared_ptr<hyy_robot_control::HyyRobotControl> hyyExternalDeviceControl;

void handle_sigint(int sig) {
    printf("\nros2 shutdown, wait seconds to exit 0.\n");
    stop.store(true);
    // hyyRobotRightArmControl->stopDeviceRun();
    hyyRobotRightArmControl->stopRobotRun();
    hyyRobotLeftArmControl->stopRobotRun();
    hyyRobotBodyControl->stopAddaxisRun();
    hyyRobotHeadControl->stopAddaxisRun();
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
    /*   Standard application structure   Client Initialize!!                   */
    /*                                                                         */
    /***************************************************************************/

    rclcpp::init(argc, argv);
    signal(SIGINT, handle_sigint);
    auto node = rclcpp::Node::make_shared("hyyRobotControl");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // hyyExternalDeviceControl = std::make_shared<hyy_robot_control::HyyRobotControl>(node);
    // if (!hyyExternalDeviceControl->init("hyy_external_device_controller", 1)){
    //     RCLCPP_ERROR(node->get_logger(), "hyydebug client initialize falied");
    //     return -1;
    // }
    
    hyyRobotLeftArmControl = std::make_shared<hyy_robot_control::HyyRobotControl>(node);
    if (!hyyRobotLeftArmControl->init("hyy_left_arm_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotLeftArmControl client initialize falied");
        return -1;
    }
    hyyRobotRightArmControl = std::make_shared<hyy_robot_control::HyyRobotControl>(node);
    if (!hyyRobotRightArmControl->init("hyy_right_arm_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotRightArmControl client initialize falied");
        return -1;
    }
    hyyRobotBodyControl = std::make_shared<hyy_robot_control::HyyRobotControl>(node);
    if (!hyyRobotBodyControl->init("hyy_body_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotBodyControl initialize falied");
        return -1;
    }
    hyyRobotHeadControl = std::make_shared<hyy_robot_control::HyyRobotControl>(node);
    if (!hyyRobotHeadControl->init("hyy_head_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotHeadControl client initialize falied");
        return -1;
    }
    executor->add_node(node);
    std::thread spinner([executor]()
        { 
            executor->spin(); 
        }
    );

    sleep(3);

    // hyyExternalDeviceControl->Gripper_initialize();

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Get Variables!!                      */
    /*                                                                         */
    /***************************************************************************/
	
    vector<int> angle_grip = {600, 700, 800, 900, 800, 500};

    string R0_VEL = "R0_v400";
    string R1_VEL = "R1_v400";
    string A0_VEL = "A0_v400";
    string A1_VEL = "A1_v400";

    string zone = "zone0";
    string tool = "tool0";
    string wobj = "wobj0";

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure   Robot Movement!!                     */
    /*                                                                         */
    /***************************************************************************/

    hyyRobotRightArmControl->isblock(false);
    hyyRobotLeftArmControl->isblock(false);
    hyyRobotBodyControl->isblock(true);
    hyyRobotHeadControl->isblock(true);

    //  STEP 1 (Initialize)
    hyyRobotBodyControl->moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    hyyRobotHeadControl->moveA("A1_J0", A1_VEL, zone, tool, wobj);
    hyyRobotLeftArmControl->moveA("R0_J0", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl->moveA("R1_J0", R1_VEL, zone, tool, wobj);
    // hyyExternalDeviceControl->Gripper_fullopen();
    // hyyExternalDeviceControl->hand_fullopen();
    blockhere(2, hyyRobotLeftArmControl->ask_status(), hyyRobotRightArmControl->ask_status());

    //  STEP 2 (Grip)
    hyyRobotBodyControl->moveA("A0_J1", A0_VEL, zone, tool, wobj);
    hyyRobotLeftArmControl->moveA("R0_J2", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl->moveA("R1_J2", R1_VEL, zone, tool, wobj);
    blockhere(2, hyyRobotLeftArmControl->ask_status(), hyyRobotRightArmControl->ask_status());
    // hyyExternalDeviceControl->Gripper_fullclose();
    // hyyExternalDeviceControl->hand_SetAngle(angle_grip);
 
    // collision test 
    // hyyRobotLeftArmControl->moveA("R0_J3", R0_VEL, zone, tool, wobj);
    // hyyRobotRightArmControl->moveA("R1_J3", R1_VEL, zone, tool, wobj);
    // blockhere(2, hyyRobotLeftArmControl->ask_status(), hyyRobotRightArmControl->ask_status());

    //  STEP 3 (Move and Drop)
    hyyRobotBodyControl->moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    hyyRobotHeadControl->moveA("A1_J1", A1_VEL, zone, tool, wobj);
    hyyRobotHeadControl->moveA("A1_J0", A1_VEL, zone, tool, wobj);
    hyyRobotBodyControl->moveA("A0_J2", A0_VEL, zone, tool, wobj);
    hyyRobotBodyControl->moveA("A0_J3", A0_VEL, zone, tool, wobj);
    // hyyExternalDeviceControl->Gripper_fullopen();
    // hyyExternalDeviceControl->hand_fullopen();

    //  STEP 4 (Back to Initial)
    hyyRobotLeftArmControl->moveA("R0_J0", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl->moveA("R1_J0", R1_VEL, zone, tool, wobj);
    blockhere(2, hyyRobotLeftArmControl->ask_status(), hyyRobotRightArmControl->ask_status());
    hyyRobotBodyControl->moveA("A0_J2", A0_VEL, zone, tool, wobj);
    hyyRobotBodyControl->moveA("A0_J0", A0_VEL, zone, tool, wobj);

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