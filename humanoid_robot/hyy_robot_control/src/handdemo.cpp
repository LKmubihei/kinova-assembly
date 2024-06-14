#include "hyy_robot_control/hyy_robot_control.h"

using namespace std;

int main(int argc, char **argv){
    
    
    //--------------------------------------------intialize---------------------------------------------//
    

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hyyRobotControl");
    hyy_robot_control::HyyRobotControl hyyHandControl(node);
    if (!hyyHandControl.init("hyy_external_device_controller", 1)){
        RCLCPP_ERROR(node->get_logger(), "hyyHandControl client initialize falied");
        return -1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::thread spinner([executor, node]()
        { 
            executor->add_node(node);
            executor->spin(); 
        }
    );

    vector<int> test, angle, force, pos, speed, angle_, pos_;
    bool if_success;
    //---------------------------------------fancy movement start---------------------------------------//

    //  STEP 1 get
    test = hyyHandControl.hand_GetAngleAct();
    RCLCPP_INFO(node->get_logger(), "AngleAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetAngleSet();
    RCLCPP_INFO(node->get_logger(), "AngleSct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetSpeedSet();
    RCLCPP_INFO(node->get_logger(), "SpeedSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetForceAct();
    RCLCPP_INFO(node->get_logger(), "ForceAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetForceSet();
    RCLCPP_INFO(node->get_logger(), "ForceSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetPosAct();
    RCLCPP_INFO(node->get_logger(), "PosAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetPosSet();
    RCLCPP_INFO(node->get_logger(), "PosSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetError();
    RCLCPP_INFO(node->get_logger(), "Error: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetCurrentAct();
    RCLCPP_INFO(node->get_logger(), "CurrentAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetTemp();
    RCLCPP_INFO(node->get_logger(), "Temp: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    //  STEP 2 set
    angle = {1000, 1000, 1000, 1000, 1000, 1000};
    angle_ = {800, 800, 800, 800, 800, 800};
    pos = {100, 100, 100, 100, 100, 100};
    pos_ = {300, 300, 300, 300, 300, 300};
    force = {400, 400, 400, 400, 400, 400};
    speed = {800, 800, 800, 800, 800, 800};

    if(hyyHandControl.hand_SetForce(force)){
        RCLCPP_INFO(node->get_logger(), "success set force!");
    }
    if(hyyHandControl.hand_SetSpeed(speed)){
        RCLCPP_INFO(node->get_logger(), "success set speed!");
    }

    if(hyyHandControl.hand_SetPos(pos)){
        RCLCPP_INFO(node->get_logger(), "success set pos!");
    }
    sleep(1);
    if(hyyHandControl.hand_SetPos(pos_)){
        RCLCPP_INFO(node->get_logger(), "success set pos!");
    }

    sleep(1);
    if(hyyHandControl.hand_SetPos(pos)){
        RCLCPP_INFO(node->get_logger(), "success set pos!");
    }
    sleep(1);
    if(hyyHandControl.hand_SetPos(pos_)){
        RCLCPP_INFO(node->get_logger(), "success set pos!");
    }

    sleep(1);
    if(hyyHandControl.hand_SetAngle(angle)){
        RCLCPP_INFO(node->get_logger(), "success set angle!");
    }
    sleep(1);
    if (hyyHandControl.hand_SetAngle(angle_))
    {
        RCLCPP_INFO(node->get_logger(), "success set angle!");
    }

    sleep(1);
    if(hyyHandControl.hand_SetAngle(angle)){
        RCLCPP_INFO(node->get_logger(), "success set angle!");
    }
    sleep(1);
    if (hyyHandControl.hand_SetAngle(angle_))
    {
        RCLCPP_INFO(node->get_logger(), "success set angle!");
    }

    //  STEP 1 get
    sleep(1);
    test = hyyHandControl.hand_GetAngleAct();
    RCLCPP_INFO(node->get_logger(), "AngleAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetAngleSet();
    RCLCPP_INFO(node->get_logger(), "AngleSct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetSpeedSet();
    RCLCPP_INFO(node->get_logger(), "SpeedSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetForceAct();
    RCLCPP_INFO(node->get_logger(), "ForceAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetForceSet();
    RCLCPP_INFO(node->get_logger(), "ForceSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetPosAct();
    RCLCPP_INFO(node->get_logger(), "PosAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);
    test = hyyHandControl.hand_GetPosSet();
    RCLCPP_INFO(node->get_logger(), "PosSet: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetError();
    RCLCPP_INFO(node->get_logger(), "Error: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetCurrentAct();
    RCLCPP_INFO(node->get_logger(), "CurrentAct: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    test = hyyHandControl.hand_GetTemp();
    RCLCPP_INFO(node->get_logger(), "Temp: %d\t%d\t%d\t%d\t%d\t%d\t", test[0], test[1], test[2], test[3], test[4], test[5]);

    //-----------------------------------------fancy movement end---------------------------------------//

    RCLCPP_INFO(node->get_logger(), "Hand finished test");
    executor->cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}