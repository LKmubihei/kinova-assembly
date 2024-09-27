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

        # 创建多个TransformStamped对象
        transforms = []

        # base_link 到 t1_base 的变换
        transform1 = self.create_transform('world', 'ur_real_base', 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        transforms.append(transform1)

        # world 到 fake_link 的变换
        # transform2 = self.create_transform('ur_real_base', 'base_link', 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        # transforms.append(transform2)

        # 通过广播器一次性发送所有静态变换
        self._static_broadcaster.sendTransform(transforms)

        self.get_logger().info('Broadcasting static transforms from base_link to t1_base and world to fake_link')

    def create_transform(self, parent_frame, child_frame, x, y, z, qx, qy, qz, qw):
        # 创建一个TransformStamped对象
        static_transform_stamped = TransformStamped()

        # 设置时间戳和坐标系ID
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = parent_frame
        static_transform_stamped.child_frame_id = child_frame

        # 设置平移
        static_transform_stamped.transform.translation.x = x
        static_transform_stamped.transform.translation.y = y
        static_transform_stamped.transform.translation.z = z

        # 设置旋转（四元数）
        static_transform_stamped.transform.rotation.x = qx
        static_transform_stamped.transform.rotation.y = qy
        static_transform_stamped.transform.rotation.z = qz
        static_transform_stamped.transform.rotation.w = qw

        return static_transform_stamped


def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
