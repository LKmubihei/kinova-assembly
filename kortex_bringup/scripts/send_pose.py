#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import MovePose Service
from hyy_message.srv import MovePose  # 假设服务名称为 MovePose

class MovePoseClient(Node):

    def __init__(self):
        super().__init__('move_pose_client')

        # 创建一个服务客户端来调用 MovePose 服务
        self.cli = self.create_client(MovePose, 'MovePose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MovePose service...')

        self.get_logger().info('MovePose service is available.')

    def call_move_pose_service(self, positionx, positiony, positionz, yaw, pitch, roll, speed):
        # 构建服务请求
        request = MovePose.Request()
        request.positionx = positionx
        request.positiony = positiony
        request.positionz = positionz
        request.yaw = yaw
        request.pitch = pitch
        request.roll = roll
        request.speed = speed

        # 异步调用服务
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"MovePose service response: {response.result}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    # 初始化 MovePoseClient 节点
    move_pose_client = MovePoseClient()

    # 示例：调用 MovePose 服务发送位姿信息
    move_pose_client.call_move_pose_service(
        positionx=0.4561,
        positiony=0.0020,
        positionz=0.4341,
        yaw=1.5700,
        pitch=0.0012,
        roll=1.5700,
        speed=0.3000
    )

    # 保持节点运行，直到服务响应完成
    rclpy.spin(move_pose_client)

    # 关闭并销毁节点
    move_pose_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
