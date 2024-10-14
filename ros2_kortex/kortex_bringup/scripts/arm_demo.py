#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ACTIONS:
from hyy_message.action import MoveXYZW  # Import the custom message type for MoveXYZW action

# Define the Pose class to store robot's position, orientation (roll, pitch, yaw), speed, and acceleration
class Pose:
    def __init__(self, positionx, positiony, positionz, roll, pitch, yaw, speed, accel):
        # Initialize pose parameters: position (x, y, z), orientation (roll, pitch, yaw), speed, and acceleration
        self.positionx = positionx
        self.positiony = positiony
        self.positionz = positionz
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.accel = accel

# Client class to send goals to the MoveXYZW action server
class MoveXYZWClient(Node):

    def __init__(self):
        # Initialize the ROS2 node named 'MoveXYZW_client'
        super().__init__('MoveXYZW_client')
        # Create an ActionClient to communicate with the MoveXYZW action server
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

        # Wait for the action server to become available
        self.get_logger().info('Waiting for MoveXYZW action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZW action server is available.')

    # Method to send a goal to the action server
    def send_goal(self, pose):
        # Validate and assign default values for speed and acceleration if they are out of bounds
        if not (0 < pose.speed <= 0.2):
            self.get_logger().warn(f'speed {pose.speed} is invalid, setting to 0.05')
            pose.speed = 0.05
        if not (0 < pose.accel <= 0.2):
            self.get_logger().warn(f'accel {pose.accel} is invalid, setting to 0.03')
            pose.accel = 0.03

        # Create a new goal message for the MoveXYZW action
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = pose.positionx
        goal_msg.positiony = pose.positiony
        goal_msg.positionz = pose.positionz
        goal_msg.yaw = pose.yaw
        goal_msg.pitch = pose.pitch
        goal_msg.roll = pose.roll
        goal_msg.speed = pose.speed
        goal_msg.accel = pose.accel

        # Log the goal details being sent to the server
        self.get_logger().info(f'Request sent: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
                               f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}, accel={goal_msg.accel}')
        
        # Send the goal asynchronously to the action server and specify a feedback callback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)  # Block until the goal is processed

        # Check if the goal was accepted by the action server
        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted.')
        # Wait for the result of the goal execution
        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        # Handle the result of the action
        result = self._get_result_future.result().result
        if result.result == "MoveXYZW:SUCCESS":
            self.get_logger().info('MoveXYZW ACTION succeeded.')
        else:
            self.get_logger().error(f'MoveXYZW ACTION failed with result: {result.result}')

    # Callback function to handle feedback during goal execution
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Log the feedback received from the action server
        self.get_logger().info(f'Received feedback: {feedback}')


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MoveXYZWClient node
    move_xyzw_client = MoveXYZWClient()

    # Define the poses that the robot should move to
    pose1 = Pose(0.4561, 0.0020, 0.4341, 1.57, 0.00119, 1.57, 0.2, 0.2)
    pose2 = Pose(0.4561, 0.0020, 0.2341, 1.57, 0.00119, 1.57, 0.2, 0.2)
    pose3 = Pose(0.4561, 0.0020, 0.6341, 1.57, 0.00119, 1.57, 0.2, 0.2)
    
    # Send goals one by one, blocking until each goal completes
    move_xyzw_client.send_goal(pose1)
    move_xyzw_client.send_goal(pose2)
    move_xyzw_client.send_goal(pose3)

    # Cleanup: Destroy the node and shutdown ROS2
    move_xyzw_client.destroy_node()
    rclpy.shutdown()

# Entry point for the program
if __name__ == '__main__':
    main()
