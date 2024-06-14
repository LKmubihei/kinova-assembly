#include "hyy_robot_control/hyy_robot_control.h"
#include <iostream>
#include <atomic>
#include <csignal>
#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

extern std::atomic<bool> stop;

void handle_sigint(int sig) {
    std::cout << "Signal " << sig << " received." << std::endl;
    stop.store(true);
    rclcpp::shutdown();
    exit(0);
}

void blockhere(int num_args, ...) {
    va_list args;
    va_start(args, num_args);

    while (1) {
        if (stop.load()) {
            std::cout << "Caught signal " << SIGINT << ", exiting..." << std::endl;
            break;
        }

        bool all_zero = true;  // 假设所有参数都为0

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
    
    /*
        ------------------------intialize------------------------
    */

    rclcpp::init(argc, argv);
    std::signal(SIGINT, handle_sigint);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hyyRobotControl");
    hyy_robot_control::HyyRobotControl hyyRobotLeftArmControl(node);
    if (!hyyRobotLeftArmControl.init("hyy_left_arm_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotLeftArmControl client initialize falied");
        return -1;
    }
    hyy_robot_control::HyyRobotControl hyyRobotRightArmControl(node);
    if (!hyyRobotRightArmControl.init("hyy_right_arm_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotRightArmControl client initialize falied");
        return -1;
    }
    hyy_robot_control::HyyRobotControl hyyRobotBodyControl(node);
    if (!hyyRobotBodyControl.init("hyy_body_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotBodyControl initialize falied");
        return -1;
    }
    hyy_robot_control::HyyRobotControl hyyRobotHeadControl(node);
    if (!hyyRobotHeadControl.init("hyy_head_controller", 0)){
        RCLCPP_ERROR(node->get_logger(), "hyyRobotHeadControl client initialize falied");
        return -1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::thread spinner([executor, node]()
        { 
            executor->add_node(node);
            executor->spin(); 
        }
    );

    string R0_VEL = "R0_v400";
    string R1_VEL = "R1_v400";
    string A0_VEL = "A0_v400";
    string A1_VEL = "A1_v400";

    string zone = "zone0";
    string tool = "tool0";
    string wobj = "wobj0";

    hyyRobotRightArmControl.isblock(false);
    hyyRobotLeftArmControl.isblock(false);
    hyyRobotBodyControl.isblock(true);
    hyyRobotHeadControl.isblock(true);

    //---------------------------------------fancy movement start---------------------------------------//

    //  STEP 1 (Back to Initial)
    // hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    // hyyRobotLeftArmControl.moveA("R0_J0", R0_VEL, zone, tool, wobj);
    // hyyRobotRightArmControl.moveA("R1_J0", R1_VEL, zone, tool, wobj);
    // blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());
    // //  STEP 2 (Press Button)
    // hyyRobotLeftArmControl.moveA("R0_J1", R0_VEL, zone, tool, wobj);
    // blockhere(1, hyyRobotLeftArmControl.ask_status());
    // hyyRobotLeftArmControl.moveA("R0_J0", R0_VEL, zone, tool, wobj);
    // hyyRobotRightArmControl.moveA("R1_J0", R1_VEL, zone, tool, wobj);
    // blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());
    // //  STEP 3 (Grip)
    // hyyRobotBodyControl.moveA("A0_J1", A0_VEL, zone, tool, wobj);
    // hyyRobotLeftArmControl.moveA("R0_J2", R0_VEL, zone, tool, wobj);
    // hyyRobotRightArmControl.moveA("R1_J2", R1_VEL, zone, tool, wobj);
    // blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());
    // //  STEP 4 (Drop)
    // hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    // hyyRobotHeadControl.moveA("A1_J1", A1_VEL, zone, tool, wobj);
    // hyyRobotHeadControl.moveA("A1_J0", A1_VEL, zone, tool, wobj);
    // hyyRobotBodyControl.moveA("A0_J2", A0_VEL, zone, tool, wobj);
    // hyyRobotBodyControl.moveA("A0_J3", A0_VEL, zone, tool, wobj);
    // //  STEP 5 (Back to Initial)
    // hyyRobotLeftArmControl.moveA("R0_J0", R0_VEL, zone, tool, wobj);
    // hyyRobotRightArmControl.moveA("R1_J0", R1_VEL, zone, tool, wobj);
    // blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());
    // hyyRobotBodyControl.moveA("A0_J2", A0_VEL, zone, tool, wobj);
    // hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);

    //  STEP 1 (Back to Initial)
    hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    hyyRobotHeadControl.moveA("A1_J0", A1_VEL, zone, tool, wobj);
    hyyRobotLeftArmControl.moveA("R0_J0", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl.moveA("R1_J0", R1_VEL, zone, tool, wobj);
    blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());

    //  STEP 2 (Grip)
    hyyRobotBodyControl.moveA("A0_J1", A0_VEL, zone, tool, wobj);
    hyyRobotLeftArmControl.moveL("R0_P2", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl.moveL("R1_P2", R1_VEL, zone, tool, wobj);
    blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());

    //  STEP 3 (Drop)
    hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);    
    hyyRobotHeadControl.moveA("A1_J1", A1_VEL, zone, tool, wobj);
    hyyRobotHeadControl.moveA("A1_J0", A1_VEL, zone, tool, wobj);
    hyyRobotBodyControl.moveA("A0_J2", A0_VEL, zone, tool, wobj);
    hyyRobotBodyControl.moveA("A0_J3", A0_VEL, zone, tool, wobj);

    //  STEP 4 (Back to Initial)
    hyyRobotLeftArmControl.moveL("R0_P0", R0_VEL, zone, tool, wobj);
    hyyRobotRightArmControl.moveL("R1_P0", R1_VEL, zone, tool, wobj);
    blockhere(2, hyyRobotLeftArmControl.ask_status(), hyyRobotRightArmControl.ask_status());
    hyyRobotBodyControl.moveA("A0_J2", A0_VEL, zone, tool, wobj);
    hyyRobotBodyControl.moveA("A0_J0", A0_VEL, zone, tool, wobj);

    //---------------------------------------fancy movement end---------------------------------------//

    RCLCPP_INFO(node->get_logger(), "Robot finished test");
    executor->cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}