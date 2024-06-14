#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <unistd.h>
#include "ccd/ccd.h"
#include "fcl/fcl.h"
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include "rclcpp/rclcpp.hpp"         
#include "std_msgs/msg/string.hpp"   
#include <hyy_message/msg/detect.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

using namespace std::chrono_literals;
using namespace fcl;
using namespace std;
using namespace Eigen;
using namespace chrono;

void get_position(KDL::Frame frame1 , KDL::Frame frame2 , Matrix<double, 3, 3>& R ,Matrix<double, 1, 3>& P ,bool l_or_r)
{
    // l_or_r 左边 0 ,右边1 
    Matrix<double,4,4> T_base_l0 ,T_base_r0,T_r0_base, T_l0_base ,T1, T2 , T_base_arm , T_base_cylinder  ;
    Matrix<double,3,3> R_base_l0 ,R_base_r0; 

    T_base_l0<<1,0,0,0.003   ,0,0.259, 0.966, 0.163  ,0,-0.966,0.259,0.999 ,    0,0,0,1 ; 
    T_base_r0<<1,0,0,0.003   ,0,0.259,-0.966,-0.163  ,0, 0.966,0.259,0.999 ,    0,0,0,1 ; 
    
    T_l0_base<<1,0,0,-0.003 ,  0,0.259,-0.966,0.923 ,    0,0.966,0.259,-0.416,       0,0,0,1;
    T_r0_base<<1,0,0,-0.003 ,  0,0.259,0.966,-0.923  ,   0, -0.966 ,0.259 ,-0.416  , 0,0,0,1; 


    R_base_l0<< 1,0,0   ,   0,0.259, 0.966   ,0,-0.966,0.259 ; 
    R_base_r0<< 1,0,0   ,   0,0.259,-0.966   ,0, 0.966,0.259 ; 

    T1<< frame1.M.data[0],frame1.M.data[1],frame1.M.data[2], (frame1.p.data[0]+frame2.p.data[0])/2 ,
         frame1.M.data[3],frame1.M.data[4],frame1.M.data[5], (frame1.p.data[1]+frame2.p.data[1])/2 ,
         frame1.M.data[6],frame1.M.data[7],frame1.M.data[8], (frame1.p.data[2]+frame2.p.data[2])/2 ,
         0,0,0,1;    
    
    if(l_or_r )
        T_base_arm = T_base_r0 ; 
    else
        T_base_arm = T_base_l0 ; 
    
    T_base_cylinder = T_base_arm *  T1 ;
    
    R<< T_base_cylinder(0,0),T_base_cylinder(0,1),T_base_cylinder(0,2)      ,T_base_cylinder(1,0),T_base_cylinder(1,1),T_base_cylinder(1,2)  
    ,T_base_cylinder(2,0),T_base_cylinder(2,1),T_base_cylinder(2,2);

    P<<T_base_cylinder(0,3),T_base_cylinder(1,3),T_base_cylinder(2,3);
}

class Collision_Detect_Node : public rclcpp::Node
{
    public:
        Collision_Detect_Node()
        : Node("collision_detect") // ROS2节点父类初始化
        {
            // 创建发布者对象（消息类型、话题名、队列长度）  自定义 detect消息类型
            publisher_ = this->create_publisher<hyy_message::msg::Detect>("/collision_detect", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Collision_Detect_Node::receive_data, this, std::placeholders::_1));

            // 创建一个定时器,定时执行回调函数
            timer_ = this->create_wall_timer(10ms, std::bind(&Collision_Detect_Node::timer_callback, this));
        }

        double joint_left[7] ,joint_right[7] ,head_joint[2] , body_joint;

    private:
          // 创建定时器周期执行的回调函数
        void timer_callback()                                                       
        {
            //测量时间 
            // auto start = std::chrono::high_resolution_clock::now();
            
           
            auto msg = hyy_message::msg::Detect();  
            // kdl 初始化 准备阶段   
            KDL::Tree left_tree , right_tree;
            std::string urdf_addr = "/home/robotck/ros2_ws/src/hyyRossPackages/multiarm_H1/h1_description/urdf/h1.urdf";
            kdl_parser::treeFromFile(urdf_addr, left_tree);
            kdl_parser::treeFromFile(urdf_addr, right_tree);
            KDL::Chain l_chain;
            left_tree.getChain("link-L0", "link-L7", l_chain);   
            KDL::Chain r_chain;
            left_tree.getChain("link-R0", "link-R7", r_chain);
            KDL::ChainFkSolverPos_recursive fk_solver_right(r_chain);
            KDL::ChainFkSolverPos_recursive fk_solver_left(l_chain);
            KDL::JntArray left_joint_positions(l_chain.getNrOfJoints());


            for (size_t i = 0; i < l_chain.getNrOfJoints(); ++i) {
                left_joint_positions(i) = joint_left[i];
            }
            KDL::JntArray right_joint_positions(r_chain.getNrOfJoints());
            for (size_t i = 0; i < r_chain.getNrOfJoints(); ++i) {
                right_joint_positions(i) = joint_right[i];
            }



            KDL::Frame left_link7_frame, left_link6_frame, left_link5_frame, left_link4_frame, left_link3_frame, left_link2_frame;
            fk_solver_left.JntToCart(left_joint_positions, left_link2_frame, 2);
            fk_solver_left.JntToCart(left_joint_positions, left_link3_frame, 3);
            fk_solver_left.JntToCart(left_joint_positions, left_link4_frame, 4);
            fk_solver_left.JntToCart(left_joint_positions, left_link5_frame, 5);
            fk_solver_left.JntToCart(left_joint_positions, left_link6_frame, 6); // 传入参数1表示获取第1个连杆的坐标系
            fk_solver_left.JntToCart(left_joint_positions, left_link7_frame, 7); 

            //求  右边 6,7连杆的位置 
            KDL::Frame right_link7_frame, right_link6_frame, right_link5_frame, right_link4_frame, right_link3_frame, right_link2_frame;
            fk_solver_right.JntToCart(right_joint_positions, right_link2_frame, 2);
            fk_solver_right.JntToCart(right_joint_positions, right_link3_frame, 3);
            fk_solver_right.JntToCart(right_joint_positions, right_link4_frame, 4);
            fk_solver_right.JntToCart(right_joint_positions, right_link5_frame, 5);
            fk_solver_right.JntToCart(right_joint_positions, right_link6_frame, 6); // 传入参数1表示获取第1个连杆的坐标系
            fk_solver_right.JntToCart(right_joint_positions, right_link7_frame, 7); 

            Matrix<double, 3, 3> R1 , R2 , R3 , L1 , L2 , L3 ,head_R ,shoulder_R ,waist_R   ;
            Matrix<double, 1, 3> P_L1, P_L2, P_L3, P_R1, P_R2, P_R3, head_P , shoulder_P ,waist_P ;

            get_position( left_link6_frame,  left_link7_frame, L3, P_L3, 0  );
            get_position( left_link4_frame,  left_link5_frame, L2, P_L2, 0  );
            get_position( left_link2_frame,  left_link3_frame, L1, P_L1, 0  );


            get_position( right_link6_frame,  right_link7_frame, R3, P_R3, 1  );
            get_position( right_link4_frame,  right_link5_frame, R2, P_R2, 1  );
            get_position( right_link2_frame,  right_link3_frame, R1, P_R1, 1  );

            head_P<<0.02,0,1.15 ;
            waist_P<<  -0.015 , 0 , 0.74 ; 
            shoulder_P<<0 ,0 ,0.995 ; 

            head_R = Eigen::MatrixXd::Identity(3,3); 
            waist_R = Eigen::MatrixXd::Identity(3,3);
            shoulder_R<< 1 ,0 ,  0,
                         0 ,0 , -1,
                         0 ,1 ,  0; 

            // 创建两个不同位置的正方体
            shared_ptr<Boxd> box1 = std::make_shared<Boxd>(0.2, 0.4, 0.4); // 头部
            shared_ptr<Boxd> box2 = std::make_shared<Boxd>(0.15, 0.2, 0.48); // 腰部
            shared_ptr<Cylinderd> cylinder1 = std::make_shared<Cylinderd>(0.06, 0.3);
            shared_ptr<Cylinderd> cylinder2 = std::make_shared<Cylinderd>(0.06, 0.22);
            shared_ptr<Cylinderd> cylinder3 = std::make_shared<Cylinderd>(0.06, 0.154);
            
            shared_ptr<Cylinderd> cylinder4 = std::make_shared<Cylinderd>(0.07, 0.5);  // 肩部的圆柱体 

            // 创建对应的碰撞对象和碰撞组
            CollisionObjectd right_arm1(cylinder1);
            CollisionObjectd right_arm2(cylinder2);
            CollisionObjectd right_arm3(cylinder3);

            CollisionObjectd left_arm1(cylinder1);
            CollisionObjectd left_arm2(cylinder2);
            CollisionObjectd left_arm3(cylinder3);

            CollisionObjectd head(box1);
            CollisionObjectd waist(box2);
            CollisionObjectd shoulder(cylinder4);


            //________给碰撞实体附上位置信息_______
            
            right_arm1.setTransform(R1,P_R1);
            right_arm2.setTransform(R2,P_R2);
            right_arm3.setTransform(R3,P_R3);

            left_arm1.setTransform(L1,P_L1);
            left_arm2.setTransform(L2,P_L2);
            left_arm3.setTransform(L3,P_L3);

            head.setTransform(head_R,head_P); 
            waist.setTransform(waist_R,waist_P);
            shoulder.setTransform(shoulder_R,shoulder_P);

            CollisionRequestd request;
            CollisionResultd result;

            DistanceRequestd requestd;
            DistanceResultd resultd;            


            //l3 --- r3 距离检测
            distance(&right_arm3, &left_arm3, requestd, resultd);
            // cout << "l3_r3_min_distance:" << resultd.min_distance << endl;
            msg.l3_r3 = result.isCollision();
            msg.l3_r3_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-r3:Collision detected!" << endl;
            result.clear();
            resultd.clear();

            //l3 --- r2 距离检测
            distance(&left_arm3,  &right_arm2, requestd, resultd);
            // cout << "l3_r2_min_distance:" << resultd.min_distance<<endl;
            msg.l3_r2 = result.isCollision();
            msg.l3_r2_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-r2:Collision detected!" << endl;            
            result.clear ();
            resultd.clear ();


            //l3 --- r1 距离检测
            distance(&left_arm3,  &right_arm1, requestd, resultd);
            // cout << "l3_r1_min_distance:" << resultd.min_distance<<endl;
            msg.l3_r1 = result.isCollision();
            msg.l3_r1_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-r1:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();


            //l2 --- r3 距离检测
            distance(&left_arm2,  &right_arm3, requestd, resultd);
            // cout << "l2_r3_min_distance:" << resultd.min_distance<<endl;
            msg.l2_r3 = result.isCollision();
            msg.l2_r3_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l2-r3:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();


            //l1 --- r3 距离检测
            distance(&left_arm1,  &right_arm3, requestd, resultd);
            // cout << "l1_r3_min_distance:" << resultd.min_distance<<endl;
            msg.l1_r3 = result.isCollision();
            msg.l1_r3_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l1-r3:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();

            //l3 --- head 距离检测
            distance(&left_arm3,  &head, requestd, resultd);
            // cout << "l3_head_min_distance:" << resultd.min_distance<<endl;
            msg.l3_head = result.isCollision();
            msg.l3_head_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-head:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();

            //l3 --- shoulder 距离检测
            distance(&left_arm3,  &shoulder, requestd, resultd);
            // cout << "l3_shoulder_min_distance:" << resultd.min_distance<<endl;
            msg.l3_shoulder = result.isCollision();
            msg.l3_shoulder_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-shoulder:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();

            //l3 --- waist 距离检测
            distance(&left_arm3,  &waist, requestd, resultd);
            // cout << "l3_waist_min_distance:" << resultd.min_distance<<endl;
            msg.l3_waist = result.isCollision();
            msg.l3_waist_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "l3-waist:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();



            //r3 --- head 距离检测
            distance(&right_arm3,  &head, requestd, resultd);
            // cout << "r3_head_min_distance:" << resultd.min_distance<<endl;
            msg.r3_head = result.isCollision();
            msg.r3_head_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "r3-head:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();

            //r3 --- shoulder 距离检测
            distance(&right_arm3,  &shoulder, requestd, resultd);
            // cout << "r3_shoulder_min_distance:" << resultd.min_distance<<endl;
            msg.r3_shoulder = result.isCollision();
            msg.r3_shoulder_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "r3-shoulder:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();

            //r3 --- waist 距离检测
            distance(&right_arm3,  &waist, requestd, resultd);
            // cout << "r3_waist_min_distance:" << resultd.min_distance<<endl;
            msg.r3_waist = result.isCollision();
            msg.r3_waist_dis = resultd.min_distance;
            if (result.isCollision()) 
                cout << "r3-waist:Collision detected!" << endl;
            result.clear ();
            resultd.clear ();


            /*测量时间 
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start  ; 
            std::cout<<"函数运行时间 "<<duration.count()<<std::endl; 
            */


            // 发布话题消息                                                 
            // RCLCPP_INFO(this->get_logger(), "Publishing"); 
            // 输出日志信息，提示已经完成话题发布   
            publisher_->publish(msg);                                                
        }

        void receive_data(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
        {

            //RCLCPP_INFO(this->get_logger(), "receive joint data");
            joint_left[0] = joint_msg->position[1];
            joint_left[1] = joint_msg->position[2];
            joint_left[2] = joint_msg->position[3];
            joint_left[3] = joint_msg->position[4];
            joint_left[4] = joint_msg->position[5];
            joint_left[5] = joint_msg->position[6];
            joint_left[6] = joint_msg->position[7];

            joint_right[0] = joint_msg->position[8];
            joint_right[1] = joint_msg->position[9];
            joint_right[2] = joint_msg->position[10];
            joint_right[3] = joint_msg->position[11];
            joint_right[4] = joint_msg->position[12];
            joint_right[5] = joint_msg->position[13];
            joint_right[6] = joint_msg->position[14];

            head_joint[0] = joint_msg->position[15];
            head_joint[1] = joint_msg->position[16];
        }

        rclcpp::TimerBase::SharedPtr timer_;                             // 定时器指针
        rclcpp::Publisher<hyy_message::msg::Detect>::SharedPtr publisher_;  // 发布者指针
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                      
{
    
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                
    
    // 创建ROS2节点对象并进行初始化          
    rclcpp::spin(std::make_shared<Collision_Detect_Node>());   
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                

    return 0;

}
