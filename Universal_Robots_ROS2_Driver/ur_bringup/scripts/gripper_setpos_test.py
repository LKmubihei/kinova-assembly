#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hyy_message.srv import Setangle  # 导入服务消息类型
import asyncio


class GripperServiceTest(Node):
    def __init__(self):
        super().__init__('gripper_service_test_node')
        # 创建服务客户端，服务名称应与服务端一致
        self.client = self.create_client(Setangle, '/gripper_cmd_Node/ur_gripper_cmd_server')

        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待机械手爪服务...')

        self.get_logger().info('机械手爪服务已可用.')

    def send_gripper_command(self, force, pos):
        """发送机械手爪服务请求"""
        # 创建请求对象
        request = Setangle.Request()
        request.status = "gripper_cmd"
        request.hand_id = 1
        request.angle = [force, pos]

        # 异步发送服务请求
        self.future = self.client.call_async(request)
        self.get_logger().info('已发送请求...')

        # 手动轮询 Future 的状态
        while not self.future.done():
            rclpy.spin_once(self, timeout_sec=0.1)  # 每次等待 0.1 秒
        
        # 处理响应
        try:
            response = self.future.result()
            if response.angle_accepted:
                self.get_logger().info('机械手爪命令成功接受.')
            else:
                self.get_logger().error('机械手爪命令被拒绝.')
            return response.angle_accepted
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)

    # 创建 GripperServiceTest 节点
    gripper_service_test = GripperServiceTest()

    # 测试：设置 force 和 pos
    force = 50  # 示例力值，范围 20-100
    pos = 500    # 示例位置值，范围 0-1000

    success = gripper_service_test.send_gripper_command(force, pos)

    if success:
        gripper_service_test.get_logger().info('机械手爪位置设置成功.')
    else:
        gripper_service_test.get_logger().error('机械手爪位置设置失败.')

    # 关闭节点
    gripper_service_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
