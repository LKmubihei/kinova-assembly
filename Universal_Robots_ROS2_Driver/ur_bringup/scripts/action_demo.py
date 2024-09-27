#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ACTIONS:
from hyy_message.action import MoveXYZW

class Pose:
    def __init__(self, positionx, positiony, positionz, roll, pitch, yaw, speed, accel):
        self.positionx = positionx
        self.positiony = positiony
        self.positionz = positionz
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.accel = accel

class MoveXYZWClient(Node):

    def __init__(self):
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

        # Wait for action server to be available
        self.get_logger().info('Waiting for MoveXYZW action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZW action server is available.')

        # Initialize an empty task queue
        self.task_queue = []

    def send_goal(self, pose):
        # 验证并分配速度值
        if not (0 < pose.speed <= 0.2):
            self.get_logger().warn(f'speed {pose.speed} 不合法,设为0.05')
            pose.speed = 0.05
        if not (0 < pose.accel <= 0.2):
            self.get_logger().warn(f'accel {pose.speed} 不合法,设为0.03')
            pose.accel = 0.03

        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = pose.positionx
        goal_msg.positiony = pose.positiony
        goal_msg.positionz = pose.positionz
        goal_msg.yaw = pose.yaw
        goal_msg.pitch = pose.pitch
        goal_msg.roll = pose.roll
        goal_msg.speed = pose.speed
        goal_msg.accel = pose.accel

        self.get_logger().info(f'发送目标:x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
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
        result = future.result().result
        if result == "MoveXYZW:SUCCESS":
            self.get_logger().info('MoveXYZW ACTION succeeded.')
            if self.task_queue:
                next_task = self.task_queue.pop(0)
                self.send_goal(next_task)
            else:
                self.get_logger().info('All tasks completed.')
                rclpy.shutdown()
        else:
            self.get_logger().error(f'MoveXYZW ACTION failed with result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    
    move_xyzw_client = MoveXYZWClient()

    # Define the poses in main
    pose1 = Pose(0.0, -0.5, 0.8, -3.1237, 0.0465, -1.58, 0.1, 0.7)
    pose2 = Pose(0.0, -0.5, 0.6, -3.1237, 0.0465, -1.58, 0.1, 0.7)
    pose3 = Pose(0.0, -0.7, 0.6, -3.1237, 0.0465, -1.58, 0.1, 0.7)

    # Add the poses to the task queue
    move_xyzw_client.task_queue = [pose1, pose2, pose3]
    
    # Send the first goal
    first_goal = move_xyzw_client.task_queue.pop(0)
    move_xyzw_client.send_goal(first_goal)

    # Keep the node running to receive feedback and results
    rclpy.spin(move_xyzw_client)

    # Cleanup and shutdown (handled in get_result_callback)
    move_xyzw_client.destroy_node()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# # Import ACTIONS:
# from hyy_message.action import MoveXYZW

# # Define GLOBAL VARIABLE -> RES:
# RES = "null"

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

# class MoveXYZWClient(Node):

#     def __init__(self):
#         super().__init__('MoveXYZW_client')
#         self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

#         # Wait for action server to be available
#         self.get_logger().info('Waiting for MoveXYZW action server...')
#         self._action_client.wait_for_server()
#         self.get_logger().info('MoveXYZW action server is available.')

#     def send_goal(self, pose):
#         # 验证并分配速度值
#         if not (0 < pose.speed <= 0.2):
#             self.get_logger().warn(f'speed {pose.speed} 不合法,设为0.05')
#             pose.speed = 0.05
#         if not (0 < pose.accel <= 0.2):
#             self.get_logger().warn(f'accel {pose.speed} 不合法,设为0.03')
#             pose.accel = 0.03

#         goal_msg = MoveXYZW.Goal()
#         goal_msg.positionx = pose.positionx
#         goal_msg.positiony = pose.positiony
#         goal_msg.positionz = pose.positionz
#         goal_msg.yaw = pose.yaw
#         goal_msg.pitch = pose.pitch
#         goal_msg.roll = pose.roll
#         goal_msg.speed = pose.speed
#         goal_msg.accel = pose.accel

#         self.get_logger().info(f'发送目标:x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
#                                f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}')
        
#         self._send_goal_future = self._action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal was rejected.')
#             rclpy.shutdown()  # Shutdown if goal was rejected
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

# def main(args=None):
#     rclpy.init(args=args)
    
#     move_xyzw_client = MoveXYZWClient()
        
#     pose1 = Pose(0.0, -0.5, 0.8, -3.1237, 0.0465, -1.58, 0.05, 0.03)
#     pose2 = Pose(0.0, -0.5, 0.6, -3.1237, 0.0465, -1.58, 0.05, 0.03)
#     pose3 = Pose(0.0, -0.7, 0.6, -3.1237, 0.0465, -1.58, 0.05, 0.03)

#     move_xyzw_client.send_goal(pose1)
#     move_xyzw_client.send_goal(pose2)
#     move_xyzw_client.send_goal(pose3)

#     # Keep the node running to receive feedback and results
#     rclpy.spin(move_xyzw_client)

#     # Cleanup and shutdown (handled in get_result_callback)
#     move_xyzw_client.destroy_node()

# if __name__ == '__main__':
#     main()




# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.action import ActionClient
# # from rclpy.node import Node

# # # Import ACTIONS:
# # from hyy_message.action import MoveXYZW

# # # Define GLOBAL VARIABLE -> RES:
# # RES = "null"

# # class MoveXYZWClient(Node):

# #     def __init__(self):
# #         super().__init__('MoveXYZW_client')
# #         self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

# #         # Wait for action server to be available
# #         self.get_logger().info('Waiting for MoveXYZW action server...')
# #         self._action_client.wait_for_server()
# #         self.get_logger().info('MoveXYZW action server is available.')

# #     def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll, speed, accel):
# #         # Validate and assign values from the function arguments
# #         if not (0 < speed <= 1):
# #             self.get_logger().warn(f'Joint speed {speed} is not valid. Must be (0,1]. Assigned: 0.01')
# #             speed = 0.01

# #         goal_msg = MoveXYZW.Goal()
# #         goal_msg.positionx = positionx
# #         goal_msg.positiony = positiony
# #         goal_msg.positionz = positionz
# #         goal_msg.yaw = yaw
# #         goal_msg.pitch = pitch
# #         goal_msg.roll = roll
# #         goal_msg.speed = speed
# #         goal_msg.accel = accel

# #         self.get_logger().info(f'Sending goal: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
# #                                f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}')
        
# #         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
# #         self._send_goal_future.add_done_callback(self.goal_response_callback)

# #     def goal_response_callback(self, future):
# #         goal_handle = future.result()
# #         if not goal_handle.accepted:
# #             self.get_logger().error('Goal was rejected.')
# #             rclpy.shutdown()  # Shutdown if goal was rejected
# #             return

# #         self.get_logger().info('Goal accepted.')
# #         self._get_result_future = goal_handle.get_result_async()
# #         self._get_result_future.add_done_callback(self.get_result_callback)

# #     def get_result_callback(self, future):
# #         global RES
# #         result = future.result().result
# #         RES = result.result
# #         if RES == "MoveXYZW:SUCCESS":
# #             self.get_logger().info(f'MoveXYZW ACTION succeeded with result: {RES}')
# #         else:
# #             self.get_logger().error(f'MoveXYZW ACTION failed with result: {RES}')

# #         # Send the next goal after receiving result
# #         if goal_queue:
# #             next_goal = goal_queue.pop(0)
# #             self.get_logger().info(f"Sending next goal: {next_goal}")
# #             self.send_goal(**next_goal)
# #         else:
# #             self.get_logger().info("All goals have been processed. Shutting down.")
# #             # After receiving the result, shut down the node
# #             rclpy.shutdown()

# #     def feedback_callback(self, feedback_msg):
# #         feedback = feedback_msg.feedback
# #         self.get_logger().info(f'Received feedback: {feedback}')

# # def main(args=None):
# #     rclpy.init(args=args)
    
# #     move_xyzw_client = MoveXYZWClient()
    
# #     # Define a queue of goals
# #     global goal_queue
# #     goal_queue = [
# #         {
# #             "positionx": 0.0,
# #             "positiony":  -0.5,
# #             "positionz":  0.8,
# #             "roll": -3.1237,
# #             "pitch": 0.0465,
# #             "yaw": -1.58,
# #             "speed": 0.05,
# #             "accel": 0.03
# #         },
# #         {
# #             "positionx": 0.0,
# #             "positiony":  -0.50,
# #             "positionz":  0.6,
# #             "roll": -3.1237,
# #             "pitch": 0.0465,
# #             "yaw": -1.58,
# #             "speed": 0.05,
# #             "accel": 0.03
# #         },
# #         {
# #             "positionx": 0.0,
# #             "positiony":  -0.7,
# #             "positionz":  0.6,
# #             "roll": -3.1237,
# #             "pitch": 0.0465,
# #             "yaw": -1.58,
# #             "speed": 0.05,
# #             "accel": 0.03
# #         },
# #         # Add more goals as needed
# #     ]
    
# #     # Send the first goal
# #     if goal_queue:
# #         first_goal = goal_queue.pop(0)
# #         move_xyzw_client.send_goal(**first_goal)

# #     # Keep the node running to receive feedback and results
# #     rclpy.spin(move_xyzw_client)

# #     # Cleanup and shutdown (handled in get_result_callback)
# #     move_xyzw_client.destroy_node()

# # if __name__ == '__main__':
# #     main()

# # import rclpy
# # from rclpy.action import ActionClient
# # from rclpy.node import Node

# # # Import ACTIONS and SERVICES:
# # from hyy_message.action import MoveXYZW
# # from hyy_message.srv import MovePose  # 假设服务名称为 MoveXYZWService

# # # Define GLOBAL VARIABLE -> RES:
# # RES = "null"


# # class MoveXYZWClient(Node):

# #     def __init__(self):
# #         super().__init__('MoveXYZW_client')
# #         self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

# #         # Wait for action server to be available
# #         self.get_logger().info('Waiting for MoveXYZW action server...')
# #         self._action_client.wait_for_server()
# #         self.get_logger().info('MoveXYZW action server is available.')

# #     def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll, speed):
# #         # Validate and assign values from the function arguments
# #         if not (0 < speed <= 1):
# #             self.get_logger().warn(f'Joint speed {speed} is not valid. Must be (0,1]. Assigned: 0.01')
# #             speed = 0.01

# #         goal_msg = MoveXYZW.Goal()
# #         goal_msg.positionx = positionx
# #         goal_msg.positiony = positiony
# #         goal_msg.positionz = positionz
# #         goal_msg.yaw = yaw
# #         goal_msg.pitch = pitch
# #         goal_msg.roll = roll
# #         goal_msg.speed = speed

# #         # self.get_logger().info(f'Sending goal: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
# #         #                        f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}')
        
# #         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
# #         self._send_goal_future.add_done_callback(self.goal_response_callback)

# #     def goal_response_callback(self, future):
# #         goal_handle = future.result()
# #         if not goal_handle.accepted:
# #             self.get_logger().error('Goal was rejected.')
# #             return

# #         self.get_logger().info('Goal accepted.')
# #         self._get_result_future = goal_handle.get_result_async()
# #         self._get_result_future.add_done_callback(self.get_result_callback)

# #     def get_result_callback(self, future):
# #         global RES
# #         result = future.result().result
# #         RES = result.result
# #         if RES == "MoveXYZW:SUCCESS":
# #             self.get_logger().info(f'MoveXYZW ACTION succeeded with result: {RES}')
# #         else:
# #             self.get_logger().error(f'MoveXYZW ACTION failed with result: {RES}')

# #     def feedback_callback(self, feedback_msg):
# #         feedback = feedback_msg.feedback
# #         self.get_logger().info(f'Received feedback: {feedback}')


# # # 服务回调函数，放在主函数外定义
# # def service_callback(request, response, move_xyzw_client):
# #     move_xyzw_client.get_logger().info(f"Received service request: x={request.positionx}, y={request.positiony}, z={request.positionz}, "
# #                                        f"roll={request.roll}, pitch={request.pitch}, yaw={request.yaw}, speed={request.speed}")
# #     # 调用 send_goal 来发送接收到的目标位姿和速度
# #     move_xyzw_client.send_goal(request.positionx, request.positiony, request.positionz, 
# #                                request.yaw, request.pitch, request.roll, request.speed)
# #     response.result = "MoveXYZW goal sent."
# #     return response


# # def main(args=None):
# #     rclpy.init(args=args)

# #     move_xyzw_client = MoveXYZWClient()

# #     # 创建服务，并传入 move_xyzw_client 实例到回调函数
# #     srv = move_xyzw_client.create_service(MovePose, 'MovePose', 
# #                                           lambda req, res: service_callback(req, res, move_xyzw_client))
# #     move_xyzw_client.get_logger().info('Service is ready to receive MoveXYZW goals.')

# #     # 一直等待服务请求
# #     rclpy.spin(move_xyzw_client)

# #     # Cleanup and shutdown
# #     move_xyzw_client.destroy_node()


# # if __name__ == '__main__':
# #     main()
