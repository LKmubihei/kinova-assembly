#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hyy_message.srv import Setangle

class GripperCmdNode(Node):
    def __init__(self):
        super().__init__('gripper_cmd_node')
        self.create_service(
            Setangle,
            '~/ur_gripper_cmd_server',
            self.controlgripper_callback
        )
        self.get_logger().info('Gripper command service is ready.')

    def controlgripper_callback(self, request, response):
        self.get_logger().info(f"Received request: status='{request.status}', force={request.angle[0]}, angle={request.angle[1]}")
        response.angle_accepted = True  # Respond positively without processing
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
