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

    // gripperHandControl = std::make_shared<hyy_control_interface::HyyExternalDevicesControl>(node);
    // if (!gripperHandControl->init("hyy_external_device_controller", 1)){
    //     RCLCPP_ERROR(node->get_logger(), "hyydebug client initialize falied");
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
    /*   Standard application structure                                        */
    /*   Get Data Test.                                                        */
    /*                                                                         */
    /***************************************************************************/

    // vector<double> cart_pos;
    // cart_pos = leftArmControl->get_cartesian_data("R0_P0");
    // RCLCPP_INFO(node->get_logger(), "R0_P0: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", cart_pos[0], cart_pos[1], cart_pos[2], cart_pos[3], cart_pos[4], cart_pos[5]);
    // cart_pos = leftArmControl->get_cartesian_data("R0_P2");
    // RCLCPP_INFO(node->get_logger(), "R0_P2: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", cart_pos[0], cart_pos[1], cart_pos[2], cart_pos[3], cart_pos[4], cart_pos[5]);

    // vector<double> joint_pos;
    // joint_pos = leftArmControl->get_joint_data("R1_J0");
    // RCLCPP_INFO(node->get_logger(), "R1_J0: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_pos[6]);
    // joint_pos = leftArmControl->get_joint_data("R1_J2");
    // RCLCPP_INFO(node->get_logger(), "R1_J2: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_pos[6]);

    // vector<double> cur_cart_pos;
    // cur_cart_pos = leftArmControl->get_cartesian_current();
    // RCLCPP_INFO(node->get_logger(), "cur_cart_pos: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", cur_cart_pos[0], cur_cart_pos[1], cur_cart_pos[2], cur_cart_pos[3], cur_cart_pos[4], cur_cart_pos[5]);

    // vector<double> cur_joint_pos;
    // cur_joint_pos = leftArmControl->get_joint_current();
    // RCLCPP_INFO(node->get_logger(), "cur_joint_pos: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", cur_joint_pos[0], cur_joint_pos[1], cur_joint_pos[2], cur_joint_pos[3], cur_joint_pos[4], cur_joint_pos[5], cur_joint_pos[6]);

    /***************************************************************************/
    /*                                                                         */
    /*   Standard application structure                                        */
    /*   Robot Move Test.                                                      */
    /*                                                                         */
    /***************************************************************************/
	
    // vector<int> angle_grip = {600, 700, 800, 900, 800, 500};

    string R0_VEL = "R0_PERCENT10";
    string R1_VEL = "R0_PERCENT10";
    string A0_VEL = "A0_PERCENT20";
    string A1_VEL = "A0_PERCENT20";

    // string zone = "DEFAULT_ZONE";
    // string tool = "DEFAULT_TOOL";
    // string wobj = "DEFAULT_WOBJ";

    leftArmControl->isblock(false);
    rightArmControl->isblock(false);
    bodyControl->isblock(true);
    headControl->isblock(true);

    int count = 0;
    while (leftArmControl->robot_ok())
    {
        //  STEP 1 (Initialize)
        bodyControl->moveA("A0_J0", A0_VEL);    
        headControl->moveA("A1_J0", A1_VEL);
        leftArmControl->moveA("R0_J0", R0_VEL);
        rightArmControl->moveA("R1_J0", R1_VEL);
        // gripperHandControl->Gripper_fullopen();
        // gripperHandControl->hand_fullopen();
        blockhere(2, leftArmControl->ask_status(), rightArmControl->ask_status());

        //  STEP 2 (Grip)
        bodyControl->moveA("A0_J1", A0_VEL);
        leftArmControl->moveJ("R0_P2", R0_VEL);
        rightArmControl->moveJ("R1_P2", R1_VEL);
        blockhere(2, leftArmControl->ask_status(), rightArmControl->ask_status());
        // gripperHandControl->Gripper_fullclose();
        // gripperHandControl->hand_SetAngle(angle_grip);

        //  STEP 3 (Move and Drop)
        bodyControl->moveA("A0_J0", A0_VEL);    
        headControl->moveA("A1_J1", A1_VEL);
        headControl->moveA("A1_J0", A1_VEL);
        bodyControl->moveA("A0_J2", A0_VEL);
        bodyControl->moveA("A0_J3", A0_VEL);
        // gripperHandControl->Gripper_fullopen();
        // gripperHandControl->hand_fullopen();

        //  STEP 4 (Back to Initial)
        leftArmControl->moveJ("R0_P0", R0_VEL);
        rightArmControl->moveJ("R1_P0", R1_VEL);
        blockhere(2, leftArmControl->ask_status(), rightArmControl->ask_status());
        bodyControl->moveA("A0_J2", A0_VEL);
        bodyControl->moveA("A0_J0", A0_VEL);

        RCLCPP_INFO(node->get_logger(), "current conut: %d.", count);
        if (++count >= 10)
        {
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