#include "hyy_robot_control/hyy_robot_control.h"
#include <iostream>
#include <atomic>
#include <csignal>
using namespace std;

std::atomic<bool> is_running{true};

void signal_handler(int signal)
{
    if (signal == SIGINT) {
        is_running.store(false, std::memory_order_relaxed);
    }
}

int main(int argc, char **argv){
    
    /*
        ------------------------intialize------------------------
    */

    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hyyRobotControl");
    hyy_robot_control::HyyRobotControl hyyRobotControl(node);
    if (!hyyRobotControl.init("hyy_arm_controller")){
        RCLCPP_ERROR(node->get_logger(), "robot control initialize falied");
        return -1;
    }

    std::vector<double> demoMovement_0;
    std::vector<double> demoMovement_1;
    std::vector<double> demoMovement_2;
    std::vector<double> demoMovement_3;
    std::vector<double> demoMovement_4;
    std::vector<std::vector<double>> demo_movement;
    vector<string> demo_movement_;
    string vel = "v100";
    string zone = "zone0";
    string tool = "tool0";
    string wobj = "wobj0";


    /*
        ------------------------check moveA ------------------------
    */

    // // Test 1.1 - moveA(vector)
    // demoMovement_0.resize(7);
    // demoMovement_1.resize(7);
    // demoMovement_2.resize(7);
    // demoMovement_3.resize(7);
    // demoMovement_4.resize(7);

    // demoMovement_0 = hyyRobotControl.get_joint_data("j0");
    // // RCLCPP_INFO(node->get_logger(), "j0: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);
    // demoMovement_1 = hyyRobotControl.get_joint_data("j1");
    // // RCLCPP_INFO(node->get_logger(), "j1: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_1[0], demoMovement_1[1], demoMovement_1[2], demoMovement_1[3], demoMovement_1[4], demoMovement_1[5], demoMovement_1[6]);
    // demoMovement_2 = hyyRobotControl.get_joint_data("j2");
    // // RCLCPP_INFO(node->get_logger(), "j2: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_2[0], demoMovement_2[1], demoMovement_2[2], demoMovement_2[3], demoMovement_2[4], demoMovement_2[5], demoMovement_2[6]);
    // demoMovement_3 = hyyRobotControl.get_joint_data("j3");
    // // RCLCPP_INFO(node->get_logger(), "j3: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_3[0], demoMovement_3[1], demoMovement_3[2], demoMovement_3[3], demoMovement_3[4], demoMovement_3[5], demoMovement_3[6]);
    // demoMovement_4 = hyyRobotControl.get_joint_data("j4");
    // // RCLCPP_INFO(node->get_logger(), "j4: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_4[0], demoMovement_4[1], demoMovement_4[2], demoMovement_4[3], demoMovement_4[4], demoMovement_4[5], demoMovement_4[6]);

    // demo_movement.push_back(demoMovement_0);
    // demo_movement.push_back(demoMovement_1);
    // demo_movement.push_back(demoMovement_2);
    // demo_movement.push_back(demoMovement_3);
    // demo_movement.push_back(demoMovement_4);
    // demo_movement.push_back(demoMovement_0);

    // RCLCPP_INFO(node->get_logger(), "Robot starts test 1.1 moveA(vector)");
    // for (size_t i = 0; i < demo_movement.size(); i++) {
    //     if (hyyRobotControl.moveA(demo_movement.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to position %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to position %ld", i);
    //         break;
    //     }
    // }

    // // Test 1.2 - moveA(string)
    // sleep(1);
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 1.2 moveA(string)");
    // demo_movement_.push_back("j0");
    // demo_movement_.push_back("j1");
    // demo_movement_.push_back("j2");
    // demo_movement_.push_back("j3");
    // demo_movement_.push_back("j4");
    // demo_movement_.push_back("j0");
    // for (size_t i = 0; i < demo_movement_.size(); i++) {
    //     if (hyyRobotControl.moveA(demo_movement_.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to position %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to position %ld", i);
    //         break;
    //     }
    // }


    /*
        ------------------------check moveL & moveJ------------------------
    */

    // hyyRobotControl.moveA("j01", vel);

    // demoMovement_0.resize(6);
    // demoMovement_1.resize(6);
    // demoMovement_2.resize(6);
    // demoMovement_3.resize(6);
    // demoMovement_4.resize(6);
    // demoMovement_0 = hyyRobotControl.get_cartesian_data("p00");
    // // RCLCPP_INFO(node->get_logger(), "p0: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
    // demoMovement_1 = hyyRobotControl.get_cartesian_data("p01");
    // // RCLCPP_INFO(node->get_logger(), "p1: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_1[0], demoMovement_1[1], demoMovement_1[2], demoMovement_1[3], demoMovement_1[4], demoMovement_1[5]);
    // demoMovement_2 = hyyRobotControl.get_cartesian_data("p02");
    // // RCLCPP_INFO(node->get_logger(), "p2: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_2[0], demoMovement_2[1], demoMovement_2[2], demoMovement_2[3], demoMovement_2[4], demoMovement_2[5]);
    // demoMovement_3 = hyyRobotControl.get_cartesian_data("p03");
    // // RCLCPP_INFO(node->get_logger(), "p3: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_3[0], demoMovement_3[1], demoMovement_3[2], demoMovement_3[3], demoMovement_3[4], demoMovement_3[5]);
    // demoMovement_4 = hyyRobotControl.get_cartesian_data("p04");
    // // RCLCPP_INFO(node->get_logger(), "p4: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_4[0], demoMovement_4[1], demoMovement_4[2], demoMovement_4[3], demoMovement_4[4], demoMovement_4[5]);

    // demo_movement.push_back(demoMovement_0);
    // demo_movement.push_back(demoMovement_1);
    // demo_movement.push_back(demoMovement_2);
    // demo_movement.push_back(demoMovement_3);
    // demo_movement.push_back(demoMovement_4);
    // demo_movement.push_back(demoMovement_0);

    // //Test 2.1 - moveL(vector)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 2.1 moveL(vector)");
    // for (size_t i = 0; i < demo_movement.size(); i++) {
    //     if (hyyRobotControl.moveL(demo_movement.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to pose %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to pose %ld", i);
    //         break;
    //     }
    // }
    // sleep(1);
    // // Test 2.2 - moveL(string)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 2.2 moveL(string)");
    // demo_movement_.push_back("p00");
    // demo_movement_.push_back("p01");
    // demo_movement_.push_back("p02");
    // demo_movement_.push_back("p03");
    // demo_movement_.push_back("p04");
    // demo_movement_.push_back("p00");
    // for (size_t i = 0; i < demo_movement_.size(); i++) {
    //     if (hyyRobotControl.moveL(demo_movement_.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to pose %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to pose %ld", i);
    //         break;
    //     }
    // }
    // sleep(1);
    // //Test 2.3 - moveJ(vector)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 2.3 moveJ(vector)");
    // for (size_t i = 0; i < demo_movement.size(); i++) {
    //     if (hyyRobotControl.moveJ(demo_movement.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to pose %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to pose %ld", i);
    //         break;
    //     }
    // }
    // sleep(1);
    // // Test 2.4 - moveJ(string)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 2.4 moveJ(string)");
    // for (size_t i = 0; i < demo_movement_.size(); i++) {
    //     if (hyyRobotControl.moveJ(demo_movement_.at(i), vel, zone, tool, wobj) == 0){
    //         RCLCPP_INFO(node->get_logger(), "Robot move to pose %ld", i);
    //     }else{
    //         RCLCPP_ERROR(node->get_logger(), "Failed to move robot to pose %ld", i);
    //         break;
    //     }
    // }

    /*
        ------------------------check moveC------------------------
    */

    // //Test 3.1 - moveC(vector)

    // hyyRobotControl.moveA("j01", vel);

    // demoMovement_0.resize(6);
    // demoMovement_1.resize(6);
    // demoMovement_2.resize(6);
    // demoMovement_3.resize(6);
    // demoMovement_4.resize(6);
    // demoMovement_0 = hyyRobotControl.get_cartesian_data("p00");
    // // RCLCPP_INFO(node->get_logger(), "p0: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
    // demoMovement_1 = hyyRobotControl.get_cartesian_data("p01");
    // // RCLCPP_INFO(node->get_logger(), "p1: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_1[0], demoMovement_1[1], demoMovement_1[2], demoMovement_1[3], demoMovement_1[4], demoMovement_1[5]);
    // demoMovement_2 = hyyRobotControl.get_cartesian_data("p02");
    // // RCLCPP_INFO(node->get_logger(), "p2: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_2[0], demoMovement_2[1], demoMovement_2[2], demoMovement_2[3], demoMovement_2[4], demoMovement_2[5]);
    // demoMovement_3 = hyyRobotControl.get_cartesian_data("p03");
    // // RCLCPP_INFO(node->get_logger(), "p3: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_3[0], demoMovement_3[1], demoMovement_3[2], demoMovement_3[3], demoMovement_3[4], demoMovement_3[5]);
    // demoMovement_4 = hyyRobotControl.get_cartesian_data("p04");
    // // RCLCPP_INFO(node->get_logger(), "p4: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_4[0], demoMovement_4[1], demoMovement_4[2], demoMovement_4[3], demoMovement_4[4], demoMovement_4[5]);

    // demo_movement.push_back(demoMovement_0);
    // demo_movement.push_back(demoMovement_1);
    // demo_movement.push_back(demoMovement_2);
    // demo_movement.push_back(demoMovement_3);
    // demo_movement.push_back(demoMovement_4);
    // demo_movement.push_back(demoMovement_0);

    // //Test 3.1 - moveC(vector)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 3.1 moveC(vector)");
    // hyyRobotControl.moveC(demo_movement.at(1), demo_movement.at(0), vel, zone, tool, wobj);
    // hyyRobotControl.moveL(demo_movement.at(2), vel, zone, tool, wobj);
    // hyyRobotControl.moveL(demo_movement.at(3), vel, zone, tool, wobj);
    // hyyRobotControl.moveC(demo_movement.at(5), demo_movement.at(4), vel, zone, tool, wobj);

    // sleep(1);
    // // Test 3.2 - moveC(string)
    // RCLCPP_INFO(node->get_logger(), "Robot starts test 3.2 moveC(string)");
    // demo_movement_.push_back("p00");
    // demo_movement_.push_back("p01");
    // demo_movement_.push_back("p02");
    // demo_movement_.push_back("p03");
    // demo_movement_.push_back("p04");
    // demo_movement_.push_back("p00");

    // hyyRobotControl.moveA("j01", vel);
    // hyyRobotControl.moveC(demo_movement_.at(1), demo_movement_.at(0), vel, zone, tool, wobj);
    // hyyRobotControl.moveL(demo_movement_.at(2), vel, zone, tool, wobj);
    // hyyRobotControl.moveL(demo_movement_.at(3), vel, zone, tool, wobj);
    // hyyRobotControl.moveC(demo_movement_.at(5), demo_movement_.at(4), vel, zone, tool, wobj);


    /*
        ------------------------check io------------------------
    */

    RCLCPP_INFO(node->get_logger(), "Robot starts test 4.1 io"); 
    for (int i = 0; i < 16; i++){
        hyyRobotControl.getDI(i);
        hyyRobotControl.setDO(i, 1);
    }

    /*
        ---------------------Comprehensive test---------------------
    */

    RCLCPP_INFO(node->get_logger(), "Robot starts Comprehensive test 5.1");
    // while(rclcpp::ok() && is_running.load(std::memory_order_relaxed)){

        hyyRobotControl.moveA("j01", vel, zone, tool, wobj);
        demoMovement_0.resize(6);
        demoMovement_0 = hyyRobotControl.get_cartesian_current();
        RCLCPP_INFO(node->get_logger(), "get_cartesian_current j00: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
        demoMovement_0.clear();
        demoMovement_0.resize(7);
        demoMovement_0 = hyyRobotControl.get_joint_current();
        RCLCPP_INFO(node->get_logger(), "get_joint_current j00: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);
    
        hyyRobotControl.moveC("p01", "p00", vel, zone, tool, wobj);
        demoMovement_0.resize(6);
        demoMovement_0 = hyyRobotControl.get_cartesian_current();
        RCLCPP_INFO(node->get_logger(), "get_cartesian_current p01: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
        demoMovement_0.clear();
        demoMovement_0.resize(7);
        demoMovement_0 = hyyRobotControl.get_joint_current();
        RCLCPP_INFO(node->get_logger(), "get_joint_current p01: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);

        hyyRobotControl.moveJ("p02", vel, zone, tool, wobj);
        demoMovement_0.resize(6);
        demoMovement_0 = hyyRobotControl.get_cartesian_current();
        RCLCPP_INFO(node->get_logger(), "get_cartesian_current p02: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
        demoMovement_0.clear();
        demoMovement_0.resize(7);
        demoMovement_0 = hyyRobotControl.get_joint_current();
        RCLCPP_INFO(node->get_logger(), "get_joint_current p02: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);

        hyyRobotControl.moveL("p03", vel, zone, tool, wobj);
        demoMovement_0.resize(6);
        demoMovement_0 = hyyRobotControl.get_cartesian_current();
        RCLCPP_INFO(node->get_logger(), "get_cartesian_current p03: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
        demoMovement_0.clear();
        demoMovement_0.resize(7);
        demoMovement_0 = hyyRobotControl.get_joint_current();
        RCLCPP_INFO(node->get_logger(), "get_joint_current p03: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);

        hyyRobotControl.moveC("p00", "p04", vel, zone, tool, wobj);
        demoMovement_0.resize(6);
        demoMovement_0 = hyyRobotControl.get_cartesian_current();
        RCLCPP_INFO(node->get_logger(), "get_cartesian_current p00: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5]);
        demoMovement_0.clear();
        demoMovement_0.resize(7);
        demoMovement_0 = hyyRobotControl.get_joint_current();
        RCLCPP_INFO(node->get_logger(), "get_joint_current p00: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", demoMovement_0[0], demoMovement_0[1], demoMovement_0[2], demoMovement_0[3], demoMovement_0[4], demoMovement_0[5], demoMovement_0[6]);
    // }

    /*
        ---------------------check grip(uncomplished)---------------------
    */

    // RCLCPP_INFO(node->get_logger(), "Robot starts test 6.1 grip");
    // hyyRobotControl.grip_create("grip");
    // hyyRobotControl.grip_control("grip", 0.5);
    // hyyRobotControl.grip_destroy("grip");

    RCLCPP_INFO(node->get_logger(), "Robot finished test");
    rclcpp::shutdown();
    return 0;
}