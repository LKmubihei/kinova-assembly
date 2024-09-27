#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ACTIONS:
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle  # 导入服务消息类型

import pyrealsense2 as rs
import numpy as np
import cv2

# 导入 tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
import time

def pixel_to_camera(u, v, K, Z):
    """将像素坐标转换为相机坐标系中的4x4齐次坐标形式"""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # 将像素坐标转换为相机坐标系中的坐标
    X_c = (u - cx) / fx * Z
    Y_c = (v - cy) / fy * Z
    Z_c = Z
    return np.array([X_c, Y_c, Z_c, 1])

# 定义全局变量 RES
RES = "null"

def tf_transformation(tf_buffer, from_frame, to_frame, node):
    """获取指定帧之间的变换关系"""
    time = rclpy.time.Time()
    count = 0
    transformation = None
    received_transformation = False
    while (not received_transformation) and (count < 100):
        try:
            transformation = tf_buffer.lookup_transform(to_frame, from_frame, time)
            node.get_logger().info(f'接收到变换: {transformation}')
            received_transformation = True
        except TransformException:
            node.get_logger().info('错误: 无法接收变换')
            rclpy.spin_once(node, timeout_sec=1.0)
            count += 1
            continue
    return transformation

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
        
        self._send_goal_future = self._action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('55555555')
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

        # # Send the next goal after receiving result
        # if goal_queue:
        #     next_goal = goal_queue.pop(0)
        #     self.get_logger().info(f"Sending next goal: {next_goal}")
        #     self.send_goal(**next_goal)
        # else:
        #     self.get_logger().info("All goals have been processed. Shutting down.")
        #     # After receiving the result, shut down the node
        #     rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

def visualmodule(base_to_flange):
    # 相机内参矩阵 ur10e
    K = np.array([[911.1848, 0.0, 646.01928],
                  [0.0, 910.3735, 358.92736],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    
    camera_to_flange = np.array([
        [8.924760722552549375e-03, -0.9995591282682333434, 0.02831779902631070475, 139.5583552397174287 / 1000],
        [0.9999530434162445802, 0.008814175836407645015, -0.004027563396773840843, -32.51499601839394415 / 1000],
        [0.003776189698006349876, 0.02835241435882007027, 0.9995908567966148572, 15.79573133794057505 / 1000],
        [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)
    
    # 配置 Realsense 流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # 启动相机流
    pipeline.start(config)

    # 创建 align 对象，用于对齐深度图和彩色图
    align_to = rs.stream.color
    align = rs.align(align_to)

    target_to_base = None  # 初始化返回值变量

    try:
        while True:
            # 获取一帧数据
            frames = pipeline.wait_for_frames()

            # 对齐深度图到彩色图
            aligned_frames = align.process(frames)

            # 获取对齐后的彩色图和深度图
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("未获取到深度帧或彩色帧")
                continue

            # 将彩色图像和深度图像转换为 numpy 数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # 转换为灰度图像
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # 高斯模糊减少噪声
            gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

            # 使用霍夫圆变换检测圆
            circles = cv2.HoughCircles(
                gray_blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=4000,
                param1=20,
                param2=40,
                minRadius=10,
                maxRadius=50
            )

            # 如果检测到圆
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # 绘制圆的外圆
                    cv2.circle(color_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # 绘制圆心
                    cv2.circle(color_image, (i[0], i[1]), 2, (0, 0, 255), 3)
                    # 输出圆心坐标
                    circle_x = i[0]
                    circle_y = i[1]
                    print("圆心坐标: ", circle_x, circle_y)
                    # 获取圆心的深度值（以米为单位）
                    depth_value = depth_frame.get_distance(i[0], i[1])
                    print('深度值: ', depth_value)
                    print(f"圆心深度值: {depth_value} 米")
                    # 计算物体在基座坐标下的位置
                    pixel_cam = pixel_to_camera(circle_x, circle_y, K, depth_value)
                    target_to_base = base_to_flange @ camera_to_flange @ pixel_cam
                    print('base_to_flange:', base_to_flange)
                    print('camera_to_flange:', camera_to_flange)
                    print('pixel_to_camera:', pixel_cam)
                    print("target_to_base:", target_to_base)
                    break  # 检测到一个圆后退出循环

            # 显示彩色图像
            cv2.imshow('Color Image', color_image)

            # 按下 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 停止相机流
        pipeline.stop()
        cv2.destroyAllWindows()
        # 返回 target_to_base
        return target_to_base

class GripperCmdClient(Node):
    def __init__(self):
        super().__init__('gripper_cmd_node')
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

    # 初始化 action 客户端
    move_xyzw_client = MoveXYZWClient()
    gripper_cmd = GripperCmdClient()
    
    # 测试：设置 force 和 pos
    force = 50  # 示例力值，范围 20-100
    pos = 730   # 示例位置值，范围 0-1000
    
    node = rclpy.create_node('tf_listener_node')
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    from_frame = 'tool0'
    to_frame = 'ur_real_base'

    base_to_flange = tf_transformation(tf_buffer, from_frame, to_frame, node)

    if base_to_flange is None:
        node.get_logger().info('获取位姿失败')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # flange在世界坐标系下的位置
    P_flange_world = np.array([base_to_flange.transform.translation.x,
                            base_to_flange.transform.translation.y,
                            base_to_flange.transform.translation.z])
    # flange在世界坐标系下的旋转矩阵
    R_flange_world = R.from_quat([
        base_to_flange.transform.rotation.x,
        base_to_flange.transform.rotation.y,
        base_to_flange.transform.rotation.z,
        base_to_flange.transform.rotation.w
    ]).as_matrix()
    
    rotation_ = R.from_matrix(R_flange_world)
    euler_in_base = rotation_.as_euler('xyz', degrees = False)
    
    # 相机在世界坐标系下的变换矩阵
    T_cam_world = np.hstack((R_flange_world, P_flange_world.reshape(-1,1)))
    T_cam_world = np.vstack((T_cam_world, np.array([0, 0, 0, 1])))
    
    # 调用相机初始化和圆心检测函数，并获取 target_to_base
    target_to_base = visualmodule(T_cam_world)

    if target_to_base is not None:
        print("从相机函数返回的 target_to_base:", target_to_base)
    else:
        print("未能获取到 target_to_base")
    
    # 定义目标队列
    global goal_queue
    goal_queue = [
        {
            "positionx": target_to_base[0],
            "positiony": target_to_base[1],
            "positionz": target_to_base[2] + 0.2,
            "roll": euler_in_base[0],
            "pitch": euler_in_base[1],
            "yaw": euler_in_base[2],
            "speed": 0.05,
            "accel": 0.03
        },
        {
            "positionx": P_flange_world[0],
            "positiony": P_flange_world[1],
            "positionz": P_flange_world[2],
            "roll": euler_in_base[0],
            "pitch": euler_in_base[1],
            "yaw": euler_in_base[2],
            "speed": 0.05,
            "accel": 0.03
        },
        {
            "positionx": target_to_base[0],
            "positiony": target_to_base[1],
            "positionz": target_to_base[2] + 0.2,
            "roll": euler_in_base[0],
            "pitch": euler_in_base[1],
            "yaw": euler_in_base[2],
            "speed": 0.05,
            "accel": 0.03
        },
        {
            "positionx": P_flange_world[0],
            "positiony": P_flange_world[1],
            "positionz": P_flange_world[2],
            "roll": euler_in_base[0],
            "pitch": euler_in_base[1],
            "yaw": euler_in_base[2],
            "speed": 0.05,
            "accel": 0.03
        } 
    ]

    #*******************#
    #    action line    #
    #*******************#

    gripper_cmd.send_gripper_command(force, 1000)
    move_xyzw_client.send_goal(**goal_queue.pop(0))
    time.sleep(0.5)
    gripper_cmd.send_gripper_command(force, 730)
    time.sleep(0.5)
    move_xyzw_client.send_goal(**goal_queue.pop(0))
    time.sleep(0.5)
    move_xyzw_client.send_goal(**goal_queue.pop(0))
    time.sleep(0.5)
    gripper_cmd.send_gripper_command(force, 1000)
    time.sleep(0.5)
    move_xyzw_client.send_goal(**goal_queue.pop(0))
    
    #**********************#
    #    action line end   #
    #**********************#

    # 保持节点运行以接收反馈和结果
    rclpy.spin(move_xyzw_client)

    # 清理并关闭节点
    move_xyzw_client.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
