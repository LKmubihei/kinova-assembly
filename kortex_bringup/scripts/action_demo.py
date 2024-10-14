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

    def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll, speed, accel):
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
        goal_msg.accel = accel

        self.get_logger().info(f'Sending goal: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
                               f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}')
        
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

        # Send the next goal after receiving result
        if goal_queue:
            next_goal = goal_queue.pop(0)
            self.get_logger().info(f"Sending next goal: {next_goal}")
            self.send_goal(**next_goal)
        else:
            self.get_logger().info("All goals have been processed. Shutting down.")
            # After receiving the result, shut down the node
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    
    move_xyzw_client = MoveXYZWClient()
    
    # Define a queue of goals
    global goal_queue
    goal_queue = [
        {
            "positionx": 0.4561,
            "positiony": 0.0020,
            "positionz": 0.4341,
            "roll": 1.57,
            "pitch": 0.00119,
            "yaw": 1.57,
            "speed": 0.1,
            "accel": 0.1
        },      
        # {
        #     "positionx": 0.38457,
        #     "positiony": -0.067957,
        #     "positionz": 0.39245,
        #     "roll": 1.572,
        #     "pitch": -0.0012,
        #     "yaw": 1.640,
        #     "speed": 0.3
        # },
        # Add more goals as needed
    ]
    
    # Send the first goal
    if goal_queue:
        first_goal = goal_queue.pop(0)
        move_xyzw_client.send_goal(**first_goal)

    # Keep the node running to receive feedback and results
    rclpy.spin(move_xyzw_client)

    # Cleanup and shutdown (handled in get_result_callback)
    move_xyzw_client.destroy_node()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# # Import ACTIONS and SERVICES:
# from hyy_message.action import MoveXYZW
# from hyy_message.srv import MovePose  # 假设服务名称为 MoveXYZWService

# # Define GLOBAL VARIABLE -> RES:
# RES = "null"


# class MoveXYZWClient(Node):

#     def __init__(self):
#         super().__init__('MoveXYZW_client')
#         self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

#         # Wait for action server to be available
#         self.get_logger().info('Waiting for MoveXYZW action server...')
#         self._action_client.wait_for_server()
#         self.get_logger().info('MoveXYZW action server is available.')

#     def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll, speed):
#         # Validate and assign values from the function arguments
#         if not (0 < speed <= 1):
#             self.get_logger().warn(f'Joint speed {speed} is not valid. Must be (0,1]. Assigned: 0.01')
#             speed = 0.01

#         goal_msg = MoveXYZW.Goal()
#         goal_msg.positionx = positionx
#         goal_msg.positiony = positiony
#         goal_msg.positionz = positionz
#         goal_msg.yaw = yaw
#         goal_msg.pitch = pitch
#         goal_msg.roll = roll
#         goal_msg.speed = speed

#         # self.get_logger().info(f'Sending goal: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
#         #                        f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}')
        
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal was rejected.')
#             return

#         self.get_logger().info('Goal accepted.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         global RES
#         result = future.result().result
#         RES = result.result
#         if RES == "MoveXYZW:SUCCESS":
#             self.get_logger().info(f'MoveXYZW ACTION succeeded with result: {RES}')
#         else:
#             self.get_logger().error(f'MoveXYZW ACTION failed with result: {RES}')

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f'Received feedback: {feedback}')


# # 服务回调函数，放在主函数外定义
# def service_callback(request, response, move_xyzw_client):
#     move_xyzw_client.get_logger().info(f"Received service request: x={request.positionx}, y={request.positiony}, z={request.positionz}, "
#                                        f"roll={request.roll}, pitch={request.pitch}, yaw={request.yaw}, speed={request.speed}")
#     # 调用 send_goal 来发送接收到的目标位姿和速度
#     move_xyzw_client.send_goal(request.positionx, request.positiony, request.positionz, 
#                                request.yaw, request.pitch, request.roll, request.speed)
#     response.result = "MoveXYZW goal sent."
#     return response


# def main(args=None):
#     rclpy.init(args=args)

#     move_xyzw_client = MoveXYZWClient()

#     # 创建服务，并传入 move_xyzw_client 实例到回调函数
#     srv = move_xyzw_client.create_service(MovePose, 'MovePose', 
#                                           lambda req, res: service_callback(req, res, move_xyzw_client))
#     move_xyzw_client.get_logger().info('Service is ready to receive MoveXYZW goals.')

#     # 一直等待服务请求
#     rclpy.spin(move_xyzw_client)

#     # Cleanup and shutdown
#     move_xyzw_client.destroy_node()


# if __name__ == '__main__':
#     main()
