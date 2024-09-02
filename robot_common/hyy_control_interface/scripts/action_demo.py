#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ACTIONS:
from hyy_message.action import MoveXYZW

# Define GLOBAL VARIABLE -> RES:
RES = "null"

class MoveXYZWClient(Node):

    def __init__(self):
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

        # Wait for action server to be available
        self.get_logger().info('Waiting for MoveXYZW action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZW action server is available.')

    def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll, speed):
        # Validate and assign values from the function arguments
        if not (0 < speed <= 1):
            self.get_logger().warn(f'Joint speed {speed} is not valid. Must be (0,1]. Assigned: 0.01')
            speed = 0.01

        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz
        goal_msg.yaw = yaw
        goal_msg.pitch = pitch
        goal_msg.roll = roll
        goal_msg.speed = speed

        self.get_logger().info(f'Sending goal: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
                               f'yaw={goal_msg.yaw}, pitch={goal_msg.pitch}, roll={goal_msg.roll}, speed={goal_msg.speed}')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected.')
            rclpy.shutdown()  # Shutdown if goal was rejected
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global RES
        result = future.result().result
        RES = result.result
        if RES == "MoveXYZW:SUCCESS":
            self.get_logger().info(f'MoveXYZW ACTION succeeded with result: {RES}')
        else:
            self.get_logger().error(f'MoveXYZW ACTION failed with result: {RES}')

        # After receiving the result, shut down the node
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    
    move_xyzw_client = MoveXYZWClient()
    
    # Define the specific values for the goal
    goal_values = {
        "positionx": 0.0,
        "positiony": 0.6,
        "positionz": 0.5,
        "yaw": 0.0,
        "pitch": 0.0, 
        "roll": -1.57,
        "speed": 0.1
    }
    
    # Send the goal with the specific values
    move_xyzw_client.send_goal(**goal_values)

    # Keep the node running to receive feedback and result
    rclpy.spin(move_xyzw_client)

    # Cleanup and shutdown (handled in get_result_callback)
    move_xyzw_client.destroy_node()

if __name__ == '__main__':
    main()
