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
    def __init__(self, position, force=50):
        self.position = position #(0-1000)
        self.force = force #(20-100)

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
        self.client = self.create_client(Setangle, '/gripper_cmd_Node/ur_gripper_cmd_server')

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
    
    move_cmd1 = Move_cmd(0.46, 0.00, 0.43, 1.57, 0.0, 1.57, 0.1, 0.2)
    move_cmd2 = Move_cmd(0.3, 0.00, 0.6, 1.57, 0.0, 1.57, 0.1, 0.2)
    move_cmd3 = Move_cmd(0.5, 0.00, 0.4, 1.57, 0.0, 1.57, 0.1, 0.2)

    gripper_open= Gripper_cmd(1000)
    gripper_grab = Gripper_cmd(0)

    #*******************#
    #    action line    #
    #*******************#

    robot_move_node.send_goal(move_cmd1)
    gripper_cmd_node.send_gripper_command(gripper_open)
    robot_move_node.send_goal(move_cmd2)
    time.sleep(1)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    time.sleep(1)
    robot_move_node.send_goal(move_cmd3)
    time.sleep(1)
    gripper_cmd_node.send_gripper_command(gripper_open)
    time.sleep(1)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    time.sleep(1)
    gripper_cmd_node.send_gripper_command(gripper_open)
    time.sleep(1)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    
    #**********************#
    #    action line end   #
    #**********************#

    # 清理并关闭节点
    robot_move_node.destroy_node()
    gripper_cmd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()