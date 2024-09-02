#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: January, 2023.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# (TBD).

# ********** ros2_execution.py ********** #
# This .py script takes the sequence of Robot Triggers defined

# Import required libraries:
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import os
import ast
import time

# Import ACTIONS:
from hyy_message.action import MoveJ
from hyy_message.action import MoveJs
from hyy_message.action import MoveL
from hyy_message.action import MoveR
from hyy_message.action import MoveXYZW
from hyy_message.action import MoveXYZ
from hyy_message.action import MoveYPR
from hyy_message.action import MoveROT
from hyy_message.action import MoveRP
from hyy_message.action import MoveG
# from ros2_grasping.action import Attacher 

# Import MESSAGES:
from hyy_message.msg import JointPose
from hyy_message.msg import JointPoseS

# Import MultiThreadedExecutor:
from rclpy.executors import MultiThreadedExecutor

# Define GLOBAL VARIABLE -> RES:
RES = "null"

# Define CLASSES for EACH ACTION:

# 4. MoveXYZW:
class MoveXYZWclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')
        # 2. Wait for MoveXYZW server to be available:
        print ("Waiting for MoveXYZW action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZW ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZWx, GoalXYZWy, GoalXYZWz, GoalXYZWyaw, GoalXYZWpitch, GoalXYZWroll, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = GoalXYZWx
        goal_msg.positionx = GoalXYZWy
        goal_msg.positionx = GoalXYZWz
        goal_msg.yaw = GoalXYZWyaw
        goal_msg.pitch = GoalXYZWpitch
        goal_msg.roll = GoalXYZWroll
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZW ACTION CALL finished.")     

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZW ACTION CALL.

# ===== INPUT PARAMETERS ===== #

# CLASS: Input program (.txt) as ROS2 PARAMETER:
PARAM_PROGRAM = "default"
P_CHECK_PROGRAM = False
class ProgramPARAM(Node):
    def __init__(self):
        global PARAM_PROGRAM
        global P_CHECK_PROGRAM
        
        super().__init__('ros2_program_param')
        self.declare_parameter('PROGRAM_FILENAME', "default")
        PARAM_PROGRAM = self.get_parameter('PROGRAM_FILENAME').get_parameter_value().string_value
        if (PARAM_PROGRAM == "default"):
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:    
            self.get_logger().info('PROGRAM_FILENAME ROS2 Parameter received: ' + PARAM_PROGRAM)
        P_CHECK_PROGRAM = True

# CLASS: Input ROBOT MODEL as ROS2 PARAMETER:
PARAM_ROBOT = "default"
P_CHECK_ROBOT = False
class RobotPARAM(Node):
    def __init__(self):
        global PARAM_ROBOT
        global P_CHECK_ROBOT
        
        super().__init__('ros2_robot_param')
        self.declare_parameter('ROBOT_MODEL', "default")
        PARAM_ROBOT = self.get_parameter('ROBOT_MODEL').get_parameter_value().string_value
        if (PARAM_ROBOT == "default"):
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('ROBOT_MODEL ROS2 Parameter received: ' + PARAM_ROBOT)

            # Check value:
            if (PARAM_ROBOT == "irb120" or 
                PARAM_ROBOT == "irb1200"or
                PARAM_ROBOT == "irb6640" or
                PARAM_ROBOT == "cr35ia" or 
                PARAM_ROBOT == "ur3" or 
                PARAM_ROBOT == "ur5" or 
                PARAM_ROBOT == "ur10" or
                PARAM_ROBOT == "panda" or
                PARAM_ROBOT == "iiwa"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The Robot model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_ROBOT = True

# CLASS: Input ROBOT End-Effector MODEL as ROS2 PARAMETER:
PARAM_EE = "default"
P_CHECK_EE = False
class eePARAM(Node):
    def __init__(self):
        global PARAM_EE
        global P_CHECK_EE
        
        super().__init__('ros2_ee_param')
        self.declare_parameter('EE_MODEL', "default")
        PARAM_EE = self.get_parameter('EE_MODEL').get_parameter_value().string_value
        if (PARAM_EE == "default"):
            self.get_logger().info('EE_MODEL ROS2 Parameter was not defined.')
            CloseProgram.CLOSE()
        else:
            self.get_logger().info('EE_MODEL ROS2 Parameter received: ' + PARAM_EE)
            
            if (PARAM_EE == "schunk" or 
                PARAM_EE == "panda_hand" or
                PARAM_EE == "none"):
                None # do nothing.
            else:
                self.get_logger().info('ERROR: The End-Effector model defined is not in the system.')
                CloseProgram.CLOSE()
        
        P_CHECK_EE = True

# CLASS: WARNING + CLOSE:
class CloseProgram():
    def CLOSE():
        print("")
        print("Please execute the program and input all ROS2 parameters in the Ubuntu Terminal as stated below:")
        print('COMMAND -> ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()

# ==================================================================================================================================== #
# ==================================================================================================================================== #
# =============================================================== MAIN =============================================================== #
# ==================================================================================================================================== #
# ==================================================================================================================================== #

def main(args=None):
    
    # Import global variable RES:
    global RES
    
    # 1. INITIALISE ROS NODE:
    rclpy.init(args=args)

    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2_RobotSimulation --> SEQUENCE EXECUTION")
    print("Python script -> ros2_execution.py")
    print("")

    # 2. INITIALISE RECEIVED ROS2 PARAMETERS:
    global PARAM_PROGRAM
    global P_CHECK_PROGRAM
    global PARAM_ROBOT
    global P_CHECK_ROBOT
    global PARAM_EE
    global P_CHECK_EE

    paramNODE = ProgramPARAM()
    while (P_CHECK_PROGRAM == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()

    robotNODE = RobotPARAM()
    while (P_CHECK_ROBOT == False):
        rclpy.spin_once(robotNODE)
    robotNODE.destroy_node()

    eeNODE = eePARAM()
    while (P_CHECK_EE == False):
        rclpy.spin_once(eeNODE)
    eeNODE.destroy_node()

    # Load components --> According to input ROS2 Parameters:

    print ("")

    # MoveJ or MoveJs, depending on de DOF of the Robot:
    # if (PARAM_ROBOT == "irb120" or 
    #     PARAM_ROBOT == "irb1200"or
    #     PARAM_ROBOT == "irb6640" or
    #     PARAM_ROBOT == "cr35ia" or 
    #     PARAM_ROBOT == "ur3" or 
    #     PARAM_ROBOT == "ur5" or 
    #     PARAM_ROBOT == "ur10"):
    #     MoveJ_CLIENT = MoveJclient()
    # elif (PARAM_ROBOT == "panda" or
    #       PARAM_ROBOT == "iiwa"):
    #     MoveJs_CLIENT = MoveJsclient()

    # # If parallel gripper: MoveG and attacher plugin activated.
    # if (PARAM_EE == "schunk" or 
    #     PARAM_EE == "panda_hand"):
    #     MoveG_CLIENT = MoveGclient()
    #     Attach_Client = ATTACHERclient()
    #     Detach_Client = DetacherPUB()

    # MoveL_CLIENT = MoveLclient()
    # MoveR_CLIENT = MoveRclient()
    MoveXYZW_CLIENT = MoveXYZWclient()
    # MoveXYZ_CLIENT = MoveXYZclient()
    # MoveYPR_CLIENT = MoveYPRclient()
    # MoveROT_CLIENT = MoveROTclient()
    # MoveRP_CLIENT = MoveRPclient()

    #  3. GET PROGRAM FILENAME:
    print("")
    print("All checks complete!")
    print("")

    # Create NODE for LOGGING:
    nodeLOG = rclpy.create_node('node_LOG')

    EXISTS = False
    PR_NAME = PARAM_PROGRAM
    filepath = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'ros2_RobotSimulation', 'ros2_execution', 'programs', PR_NAME + ".txt")
    EXISTS = os.path.exists(filepath)
    if (EXISTS == True):
        print(PR_NAME + " file found! Executing program...")
        nodeLOG.get_logger().info("SUCCESS: " + PR_NAME + " file (program) found.")
        time.sleep(1)
    elif (EXISTS == False):
        print(PR_NAME + " file not found. Please input the PROGRAM FILENAME correctly as a ROS2 parameter in the Ubuntu Terminal:")
        nodeLOG.get_logger().info("ERROR: " + PR_NAME + " file (program) not found. Please try again.")
        print('COMMAND -> ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"')
        print("Closing... BYE!")
        time.sleep(5)
        exit()

    # OPEN PR_NAME.txt FILE:
    with open(filepath) as file:
        f = file.readlines()
        i = 1
        seq = dict()
        for line in f:
            seq[str(i)] = ast.literal_eval(line)
            i = i + 1
        file.close()

    # Log number of steps:
    nodeLOG.get_logger().info(PR_NAME + ": Number of steps -> " + str(len(seq)))
    time.sleep(1)

    # ================================= 4. SEQUENCE ================================= #
    for i in range (1, len(seq)+1):
        
        trigger = seq[str(i)]
        
        if (trigger['action'] == 'MoveJ'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveJ:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            JP = JointPose()
            JP.joint1 = trigger['value']['joint1']
            JP.joint2 = trigger['value']['joint2']
            JP.joint3 = trigger['value']['joint3']
            JP.joint4 = trigger['value']['joint4']
            JP.joint5 = trigger['value']['joint5']
            JP.joint6 = trigger['value']['joint6']
            MoveJ_CLIENT.send_goal(JP, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveJ_CLIENT)
                if (RES != "null"):
                    break

            print ("Result of MoveJ ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveJ:SUCCESS"):
                print("MoveJ ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveJ ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveJ ACTION in step number -> " + str(i) + " failed.")
                break
        
        elif (trigger['action'] == 'MoveJs'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveJs:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            JPS = JointPoseS()
            JPS.joint1 = trigger['value']['joint1']
            JPS.joint2 = trigger['value']['joint2']
            JPS.joint3 = trigger['value']['joint3']
            JPS.joint4 = trigger['value']['joint4']
            JPS.joint5 = trigger['value']['joint5']
            JPS.joint6 = trigger['value']['joint6']
            JPS.joint7 = trigger['value']['joint7']
            MoveJs_CLIENT.send_goal(JPS, JointSPEED)

            while rclpy.ok():
                rclpy.spin_once(MoveJs_CLIENT)
                if (RES != "null"):
                    break

            print ("Result of MoveJs ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveJs:SUCCESS"):
                print("MoveJs ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveJs ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveJs ACTION in step number -> " + str(i) + " failed.")
                break
            
        elif (trigger['action'] == 'MoveL'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveL:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            MoveX = trigger['value']['movex']
            MoveY = trigger['value']['movey']
            MoveZ = trigger['value']['movez']
            MoveL_CLIENT.send_goal(MoveX,MoveY,MoveZ, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveL_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveL ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveL:SUCCESS"):
                print("MoveL ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveL ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveL ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'MoveR'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveR:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            joint = trigger['value']['joint']
            value = trigger['value']['value']
            MoveR_CLIENT.send_goal(joint,value, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveR_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveR ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveR:SUCCESS"):
                print("MoveR ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveR ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveR ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'MoveXYZW'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveXYZW:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            
            MoveXYZW_CLIENT.send_goal(positionx,positiony,positionz,yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZW_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZW ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZW:SUCCESS"):
                print("MoveXYZW ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZW ACTION in step number -> " + str(i) + " failed.")
                break

        elif (trigger['action'] == 'MoveXYZ'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveXYZ:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            positionx = trigger['value']['positionx']
            positiony = trigger['value']['positiony']
            positionz = trigger['value']['positionz']
            MoveXYZ_CLIENT.send_goal(positionx,positiony,positionz, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveXYZ_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveXYZ ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveXYZ:SUCCESS"):
                print("MoveXYZ ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveXYZ ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        elif (trigger['action'] == 'MoveYPR'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveYPR:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            MoveYPR_CLIENT.send_goal(yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveYPR_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveYPR ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveYPR:SUCCESS"):
                print("MoveYPR ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveYPR ACTION in step number -> " + str(i) + " failed.")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveYPR ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        elif (trigger['action'] == 'MoveROT'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveROT:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            MoveROT_CLIENT.send_goal(yaw,pitch,roll, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveROT_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveROT ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveROT:SUCCESS"):
                print("MoveROT ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveROT ACTION in step number -> " + str(i) + " failed.")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveROT ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break
        
        elif (trigger['action'] == 'MoveRP'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> MoveRP:")
            print(trigger['value'])

            # Joint SPEED:
            JointSPEED = trigger['speed']
            if (JointSPEED <= 0 or JointSPEED > 1):
                print ("Joint speed -> " + str(JointSPEED) + " not valid. Must be (0,1]. Assigned: 0.01")
                JointSPEED = 0.01
            else:
                print("Joint speed -> " + str(JointSPEED))

            yaw = trigger['value']['yaw']
            pitch = trigger['value']['pitch']
            roll = trigger['value']['roll']
            x = trigger['value']['x']
            y = trigger['value']['y']
            z = trigger['value']['z']
            MoveRP_CLIENT.send_goal(yaw,pitch,roll,x,y,z, JointSPEED)
            
            while rclpy.ok():
                rclpy.spin_once(MoveRP_CLIENT)
                if (RES != "null"):
                    break
            
            print ("Result of MoveRP ACTION CALL is -> { " + RES + " }")
            
            if (RES == "MoveRP:SUCCESS"):
                print("MoveRP ACTION in step number -> " + str(i) + " successfully executed.")
                RES = "null"
            else:
                print("MoveRP ACTION in step number -> " + str(i) + " failed.")
                nodeLOG.get_logger().info("ERROR: Program finished since MoveRP ACTION in step number -> " + str(i) + " failed.")
                print("The program will be closed. Bye!")
                break

        elif (trigger['action'] == 'Attach'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> ATTACH OBJECT:")
            print(trigger['value'])

            OBJ = trigger['value']['object']
            EE = trigger['value']['endeffector']
            
            Attach_Client.send_goal(OBJ,EE)
            rclpy.spin_once(Attach_Client)
            
            print("Object ATTACHED successfully.")

        elif (trigger['action'] == 'Detach'):
            
            print("")
            print("STEP NUMBER " + str(i) + " -> DETACH OBJECT:")
            print(trigger['value'])

            OBJ = trigger['value']['object']
            
            MSG = String()
            MSG.data = "True"

            t_end = time.time() + 1
            while time.time() < t_end:
                Detach_Client.publisher_.publish(MSG) # Publish repeatedly for a second to make sure that the ATTACHER SERVER receives the message.
        
            print("Object DETACHED successfully.")

        elif (trigger['action'] == 'GripperOpen'):

            if (PARAM_EE == "schunk"):
            
                print("")
                print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

                GP = 0.0
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

            elif (PARAM_EE == "panda_hand"):

                print("")
                print("STEP NUMBER " + str(i) + " -> GripperOpen (MoveG).")

                GP = 0.035
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

        elif (trigger['action'] == 'GripperClose'):

            if (PARAM_EE == "schunk"):
            
                print("")
                print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

                GP = 0.005
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

            elif (PARAM_EE == "panda_hand"):
                
                print("")
                print("STEP NUMBER " + str(i) + " -> GripperClose (MoveG).")

                GP = 0.0
                MoveG_CLIENT.send_goal(GP)
                
                while rclpy.ok():
                    rclpy.spin_once(MoveG_CLIENT)
                    if (RES != "null"):
                        break
                
                print ("Result of MoveG ACTION CALL is -> { " + RES + " }")
                
                if (RES == "MoveG:SUCCESS"):
                    print("MoveG ACTION in step number -> " + str(i) + " successfully executed.")
                    RES = "null"
                else:
                    print("MoveG ACTION in step number -> " + str(i) + " failed.")
                    print("The program will be closed. Bye!")
                    break

        else:
            print("Step number " + str(i) + " -> Action type not identified. Please check.")
            print("The program will be closed. Bye!")
            nodeLOG.get_logger().info("ERROR: Program finished since ACTION NAME in step number -> " + str(i) + " was not identified.")
            break

        #time.sleep(1)

    print("")
    print("SEQUENCE EXECUTION FINISHED!")
    print("Program will be closed. Bye!")
    nodeLOG.get_logger().info("SUCESS: Program execution sucessfully finished.")
    nodeLOG.destroy_node()
    print("Closing... BYE!")
    time.sleep(5)
        

if __name__ == '__main__':
    main()