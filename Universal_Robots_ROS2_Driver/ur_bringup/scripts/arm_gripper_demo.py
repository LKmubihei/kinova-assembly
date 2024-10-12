#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import the action and service definitions
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle 

import time
import signal
import sys

# Class to represent a robot pose with position, orientation, speed, and acceleration
class Move_cmd:
    def __init__(self, positionx, positiony, positionz, roll, pitch, yaw, speed, accel):
        self.positionx = positionx
        self.positiony = positiony
        self.positionz = positionz
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.accel = accel

# Class to represent a gripper status
class Gripper_cmd:
    def __init__(self, force, position):
        self.force = force #(20-100)
        self.position = position #(0-1000)

# Client class to send MoveXYZW action goals
class MoveXYZWClient(Node):

    def __init__(self):
        super().__init__('MoveXYZW_client')
        # Create action client to communicate with MoveXYZW action server
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

        # Wait for action server to be available
        self.get_logger().info('Waiting for MoveXYZW action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZW action server is available.')

    def send_goal(self, move_cmd):
        # Validate speed and acceleration values
        if not (0 < move_cmd.speed <= 0.2):
            self.get_logger().warn(f'speed {move_cmd.speed} is invalid, setting to 0.05')
            move_cmd.speed = 0.05
        if not (0 < move_cmd.accel <= 0.2):
            self.get_logger().warn(f'accel {move_cmd.accel} is invalid, setting to 0.03')
            move_cmd.accel = 0.03

        # Prepare the goal message
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = move_cmd.positionx
        goal_msg.positiony = move_cmd.positiony
        goal_msg.positionz = move_cmd.positionz
        goal_msg.yaw = move_cmd.yaw
        goal_msg.pitch = move_cmd.pitch
        goal_msg.roll = move_cmd.roll
        goal_msg.speed = move_cmd.speed
        goal_msg.accel = move_cmd.accel

        # Log the goal details
        self.get_logger().info(f'Request sent: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
                               f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}, accel={goal_msg.accel}')
        
        # Send the goal asynchronously and wait for the result
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        # Get the result of the goal request
        self._goal_handle = self._send_goal_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        # Handle the result of the action
        result = self._get_result_future.result().result
        if result.result == "MoveXYZW:SUCCESS":
            self.get_logger().info('MoveXYZW ACTION succeeded.')
        else:
            self.get_logger().error(f'MoveXYZW ACTION failed with result: {result.result}')

    # Callback to handle feedback received during the execution of the action
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

# Client class to send gripper commands via a service
class GripperCmdClient(Node):
    def __init__(self):
        super().__init__('gripper_cmd_client')
        # Create service client to communicate with the gripper command service
        self.client = self.create_client(Setangle, '/gripper_cmd_node/ur_gripper_cmd_server')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        self.get_logger().info('Gripper service server is available.')

    # Method to send a command to the gripper (force and position)
    def send_gripper_command(self, gripper_cmd):
        """Send gripper service request"""
        # Create request object
        request = Setangle.Request()
        request.status = "gripper_cmd"
        request.hand_id = 1
        request.angle = [gripper_cmd.force, gripper_cmd.position]

        # Asynchronously send the service request
        self.future = self.client.call_async(request)
        self.get_logger().info(f"Request sent: status = {request.status}, force = {request.angle[0]}, angle = {request.angle[1]}")

        # Manually poll the future's status and wait for a response
        while not self.future.done():
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait 0.1 second each time
        
        # Process the service response
        try:
            response = self.future.result()
            if response.angle_accepted:
                self.get_logger().info('Gripper command accepted successfully.')
            else:
                self.get_logger().error('Gripper command was rejected.')
            return response.angle_accepted
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False

def main(args=None):

    rclpy.init(args=args)

    # 初始化 action 客户端
    robot_move_node = MoveXYZWClient()
    gripper_cmd_node = GripperCmdClient()

    move_cmd1 = Move_cmd(0.0, -0.4, 0.9, -3.1237, 0.0465, -1.58, 0.1, 0.2)
    move_cmd2 = Move_cmd(0.0, -0.4, 0.5, -3.1237, 0.0465, -1.58, 0.1, 0.2)
    move_cmd3 = Move_cmd(0.0, -0.8, 0.9, -3.1237, 0.0465, -1.58, 0.1, 0.2)

    gripper_open= Gripper_cmd(50, 1000)
    gripper_grab = Gripper_cmd(50, 730)

    #*******************#
    #    action line    #
    #*******************#

    robot_move_node.send_goal(move_cmd1)
    gripper_cmd_node.send_gripper_command(gripper_open)
    robot_move_node.send_goal(move_cmd2)
    time.sleep(2)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    time.sleep(2)
    robot_move_node.send_goal(move_cmd3)
    time.sleep(2)
    gripper_cmd_node.send_gripper_command(gripper_open)
    
    #**********************#
    #    action line end   #
    #**********************#

    # 清理并关闭节点
    robot_move_node.destroy_node()
    gripper_cmd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# # Import the action and service definitions
# from hyy_message.action import MoveXYZW
# from hyy_message.srv import Setangle 

# import time
# import signal
# import sys

# # Class to represent a robot pose with position, orientation, speed, and acceleration
# class Pose:
#     def __init__(self, positionx, positiony, positionz, roll, pitch, yaw, speed, accel):
#         self.positionx = positionx
#         self.positiony = positiony
#         self.positionz = positionz
#         self.roll = roll
#         self.pitch = pitch
#         self.yaw = yaw
#         self.speed = speed
#         self.accel = accel

# # Client class to send MoveXYZW action goals
# class MoveXYZWClient(Node):

#     def __init__(self):
#         super().__init__('MoveXYZW_client')
#         # Create action client to communicate with MoveXYZW action server
#         self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

#         # Wait for action server to be available
#         self.get_logger().info('Waiting for MoveXYZW action server...')
#         self._action_client.wait_for_server()
#         self.get_logger().info('MoveXYZW action server is available.')
#         self.next_goal = True

#     def send_goal(self, pose, delay_time = 0.0):
        
#         while not self.next_goal:
#             # self.get_logger().info('Waiting for next_goal to be ready...')
#             rclpy.spin_once(self)

#         # Validate speed and acceleration values
#         if not (0 < pose.speed <= 0.2):
#             self.get_logger().warn(f'speed {pose.speed} is invalid, setting to 0.05')
#             pose.speed = 0.05
#         if not (0 < pose.accel <= 0.2):
#             self.get_logger().warn(f'accel {pose.accel} is invalid, setting to 0.03')
#             pose.accel = 0.03

#         # Prepare the goal message
#         self.goal_msg = MoveXYZW.Goal()
#         self.goal_msg.positionx = pose.positionx
#         self.goal_msg.positiony = pose.positiony
#         self.goal_msg.positionz = pose.positionz
#         self.goal_msg.yaw = pose.yaw
#         self.goal_msg.pitch = pose.pitch
#         self.goal_msg.roll = pose.roll
#         self.goal_msg.speed = pose.speed
#         self.goal_msg.accel = pose.accel
#         self.delay_time = delay_time

#         # Log the goal details
#         self.get_logger().info(f'Request sent: x={self.goal_msg.positionx}, y={self.goal_msg.positiony}, z={self.goal_msg.positionz}, '
#                                f'roll={self.goal_msg.roll}, pitch={self.goal_msg.pitch}, yaw={self.goal_msg.yaw}, speed={self.goal_msg.speed}, accel={self.goal_msg.accel}')
        
#         # Send the goal asynchronously and set up callbacks for response and result
#         self._send_goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
#         self.next_goal = False
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         # Get the result of the goal request
#         self._goal_handle = future.result()
#         if not self._goal_handle.accepted:
#             self.get_logger().error('Goal was rejected.')
#             return

#         self.get_logger().info('Goal accepted.')
#         self._get_result_future = self._goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         # Handle the result of the action
#         result = future.result().result
#         if result.result == "MoveXYZW:SUCCESS":
#             self.get_logger().info('MoveXYZW ACTION succeeded.')
#             self.next_goal = True
#             time.sleep(self.delay_time)
#         else:
#             self.get_logger().error(f'MoveXYZW ACTION failed with result: {result.result}')

#     # Callback to handle feedback received during the execution of the action
#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f'Received feedback: {feedback}')

#     def cancel_goal(self):
#         if self._goal_handle is not None:
#             self.get_logger().info('Sending cancel request...')
#             future = self._goal_handle.cancel_goal_async()
#             self.get_logger().info('Sent cancel request...')
#             future.add_done_callback(self.cancel_done_callback)
#         else:
#             self.get_logger().error('No goal handle found to cancel.')

#     def cancel_done_callback(self, future):
#         cancel_response = future.result()
#         if cancel_response is not None and cancel_response.accepted:
#             self.get_logger().info('Goal successfully cancelled.')
#         else:
#             self.get_logger().error('Goal cancellation failed.')

# # Client class to send gripper commands via a service
# class GripperCmdClient(Node):
#     def __init__(self):
#         super().__init__('gripper_cmd_client')
#         # Create service client to communicate with the gripper command service
#         self.client = self.create_client(Setangle, '/gripper_cmd_node/ur_gripper_cmd_server')

#         # Wait for the service to be available
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for gripper service...')

#         self.get_logger().info('Gripper service server is available.')

#     # Method to send a command to the gripper (force and position)
#     def send_gripper_command(self, force, pos, prefix_delay_time = 3, suffix_delay_time = 3):
        
#         time.sleep(prefix_delay_time)

#         """Send gripper service request"""
#         # Create request object
#         request = Setangle.Request()
#         request.status = "gripper_cmd"
#         request.hand_id = 1
#         request.angle = [force, pos]

#         # Asynchronously send the service request
#         self.future = self.client.call_async(request)
#         self.get_logger().info(f"Request sent: status = {request.status}, force = {request.angle[0]}, angle = {request.angle[1]}")

#         # Manually poll the future's status and wait for a response
#         while not self.future.done():
#             rclpy.spin_once(self, timeout_sec=0.1)  # Wait 0.1 second each time
        
#         # Process the service response
#         try:
#             response = self.future.result()
#             if response.angle_accepted:
#                 self.get_logger().info('Gripper command accepted successfully.')
#             else:
#                 self.get_logger().error('Gripper command was rejected.')
#             time.sleep(suffix_delay_time)
#             return response.angle_accepted
#         except Exception as e:
#             self.get_logger().error(f'Service call failed: {e}')
#             return False

# def main(args=None):

#     rclpy.init(args=args)

#     # 初始化 action 客户端
#     robot_move_node = MoveXYZWClient()
#     gripper_cmd_node = GripperCmdClient()
    
#     # 捕捉 Ctrl-C 信号
#     def signal_handler(sig, frame):
#         robot_move_node.get_logger().info('Ctrl-C detected.')
#         robot_move_node.cancel_goal()
#         rclpy.spin_once(robot_move_node, timeout_sec=0.1)

#     # 注册信号处理函数
#     signal.signal(signal.SIGINT, signal_handler)
    
#     # 测试：设置 force 和 pos
#     force = 50  # 示例力值，范围 20-100
#     pos = 730   # 示例位置值，范围 0-1000
#     time_ = 2

#     pose1 = Pose(0.0, -0.4, 0.9, -3.1237, 0.0465, -1.58, 0.1, 0.2)
#     pose2 = Pose(0.0, -0.4, 0.5, -3.1237, 0.0465, -1.58, 0.1, 0.2)
#     pose3 = Pose(0.0, -0.8, 0.9, -3.1237, 0.0465, -1.58, 0.1, 0.2)

#     #*******************#
#     #    action line    #
#     #*******************#

#     robot_move_node.send_goal(pose1)
#     gripper_cmd_node.send_gripper_command(force, 1000)
#     robot_move_node.send_goal(pose2)
#     gripper_cmd_node.send_gripper_command(force, 730)
#     robot_move_node.send_goal(pose3)
#     gripper_cmd_node.send_gripper_command(force, 1000)
    
#     #**********************#
#     #    action line end   #
#     #**********************#

#     # 保持节点运行以接收反馈和结果
#     # rclpy.spin(robot_move_node)

#     # 清理并关闭节点
#     robot_move_node.destroy_node()
#     gripper_cmd_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()