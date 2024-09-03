#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFBroadcaster(Node):

    def __init__(self):
        super().__init__('static_tf_broadcaster')

        # 创建静态变换广播器
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # 创建一个TransformStamped对象
        static_transform_stamped = TransformStamped()

        # 设置时间戳和坐标系ID
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_link'
        static_transform_stamped.child_frame_id = 'h1_base'

        # 设置平移
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.52

        # 设置旋转（四元数）
        static_transform_stamped.transform.rotation.x = -0.707
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 0.707

        # 通过广播器发送静态变换
        self._static_broadcaster.sendTransform(static_transform_stamped)

        self.get_logger().info('Broadcasting static transform from base_link to h1_base')

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
