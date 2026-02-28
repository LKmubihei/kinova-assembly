#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
# Import ACTIONS and SERVICES
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle

# Global variable to store transformation
RES = "null"

# Function to convert pixel coordinates to camera coordinates
def pixel_to_camera(u, v, K, Z):
    """Convert pixel coordinates to camera coordinates in 4x4 homogeneous form."""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Convert pixel coordinates to camera coordinates
    X_c = (u - cx) / fx * Z
    Y_c = (v - cy) / fy * Z
    Z_c = Z
    return np.array([X_c, Y_c, Z_c, 1])

def create_homogeneous_transform(translation_vector, rotation_matrix):

    # Ensure the translation vector is a column vector
    translation_vector = np.array(translation_vector).reshape(-1, 1)
    
    # Horizontally stack the rotation matrix and translation vector
    T_3x4 = np.hstack((rotation_matrix, translation_vector))
    
    # Vertically stack the bottom row [0, 0, 0, 1] to create a 4x4 matrix
    T_4x4 = np.vstack((T_3x4, np.array([0, 0, 0, 1])))

    return T_4x4

class GGCNNService:
    def __init__(self, node, namespace: str = "/camera/d435"):
        # 初始化节点
        self.node = node

        # 初始化订阅者
        self.rgb_sub = self.node.create_subscription(
            Image, f'{namespace}/color/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.node.create_subscription(
            Image, f'{namespace}/depth/image_raw', self.depth_callback, 1)
        self.depth_info_sub = self.node.create_subscription(
            CameraInfo, f'{namespace}/depth/camera_info', self.depth_info_callback, 1)

        self.node.get_logger().info(f'Initialized {namespace} subscribers')

        # 相机参数和接收标志
        self.K_depth = []  # 内参矩阵
        self.depth_scale = 1000
        self.h_depth = 0
        self.w_depth = 0
        self.received_depth = False
        self.received_rgb = False
        self.received_depth_K = False
        self.rgb = None
        self.depth = None

        # CV Bridge
        self.cv_bridge = CvBridge()

    def rgb_callback(self, img_msg):
        self.received_rgb = False
        self.rgb = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.received_rgb = True

    def depth_callback(self, img_msg):
        self.received_depth = False
        self.depth = self.cv_bridge.imgmsg_to_cv2(img_msg, "32FC1")  # 深度图像默认32位浮点型
        self.h_depth = img_msg.height
        self.w_depth = img_msg.width
        self.received_depth = True

    def depth_info_callback(self, info_msg):
        self.received_depth_K = False
        self.K_depth = np.array(info_msg.k).reshape(3, 3)
        self.received_depth_K = True

    def show_image(self, img, title):
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype('uint8')
        cv2.imwrite(title, img)
        self.node.get_logger().info(f"Image saved as {title}")

class FrameTransformListener:
    def __init__(self, node: Node, from_frame: str = "camera_depth_frame", to_frame: str = "base_link_inertia", timer_period: float = 1.0):
        """
        封装一个类用于实时监听并获取两个frame之间的坐标变换。
        
        :param node: 已初始化的ROS2节点对象
        :param from_frame: 源坐标系
        :param to_frame:   目标坐标系
        :param timer_period: 定时器触发的时间间隔（秒）
        """
        self.node = node
        self.from_frame = from_frame
        self.to_frame = to_frame

        # 创建tf2 Buffer及Listener用于监听TF变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.node.get_logger().info(f"Initializing FrameTransformListener for {self.from_frame} -> {self.to_frame}")

        # 创建定时器
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

        self.transformation =None

    def timer_callback(self):
        """
        定时器回调函数，用于获取并处理坐标变换。
        """
        try:
            # 尝试获取最新的变换
            self.transformation = self.tf_buffer.lookup_transform(self.to_frame, self.from_frame, rclpy.time.Time())
            # self.node.get_logger().info(f"Received transformation: {self.transformation}")
        except TransformException as e:
            self.node.get_logger().warn(f"Failed to get transformation: {e}")

def process_transformation(transformation: TransformStamped):
    """
    处理获取到的变换，包括提取平移向量、旋转矩阵和欧拉角，并生成齐次变换矩阵。
    
    :param transformation: 获取到的 TransformStamped 对象
    :return: 平移向量 (P_), 旋转矩阵 (R_), 欧拉角 (Euler_), 齐次变换矩阵 (T_)
    """
    try:
        # 提取平移向量
        P_ = np.array([
            transformation.transform.translation.x,
            transformation.transform.translation.y,
            transformation.transform.translation.z
        ])

        # 提取旋转四元数并转换为旋转矩阵
        R_ = R.from_quat([
            transformation.transform.rotation.x,
            transformation.transform.rotation.y,
            transformation.transform.rotation.z,
            transformation.transform.rotation.w
        ]).as_matrix()

        # 将旋转矩阵转换为欧拉角（XYZ顺序）
        Euler_ = R.from_matrix(R_).as_euler('xyz', degrees=False)

        # 创建齐次变换矩阵
        T_ = create_homogeneous_transform(P_, R_)

        # 打印结果
        print("位置 (Translation Vector):\n", P_)
        print("欧拉角 (Euler Angles):\n", Euler_)
        print("旋转矩阵 (Rotation Matrix):\n", R_)
        print("变换矩阵 (Transform Matrix):\n", T_)

        return P_, R_, Euler_, T_

    except Exception as e:
        print(f"Error processing transformation: {e}")
        return None, None, None, None

# Visual module to handle camera input and circle detection
def visualmodule(camera_data, camera_Transfor_in_base):
    if not (camera_data.received_rgb and camera_data.received_depth and camera_data.received_depth_K):
        print("Waiting for RGB, Depth data and Camera Info...")
        return None
    color_image = camera_data.rgb
    depth_image = camera_data.depth
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
        gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=100
    )
    
    target_to_base = None
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(color_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(color_image, (i[0], i[1]), 2, (0, 0, 255), 3)
            print(f"Color Image Dimensions: {color_image.shape}")
            print(f"Depth Image Dimensions: {depth_image.shape}")
            depth_value = depth_image[i[1], i[0]]/camera_data.depth_scale
            pixel_cam = pixel_to_camera(i[0], i[1], camera_data.K_depth, depth_value)
            target_to_base = camera_Transfor_in_base @ pixel_cam
            print(f"camera_data.depth_scale:\n {camera_data.depth_scale} ")
            print(f"camera_data.K_depth:\n {camera_data.K_depth} ")
            print("Circle center:", (i[0], i[1]))
            print(f"pixel_cam: {pixel_cam} ")
            print(f"Depth value: {depth_value} m")
            print(f"Grasp point: {target_to_base}")
            break

    cv2.imshow('Color Image', color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return target_to_base

    return None

# Class to represent a robot pose with position, orientation, speed, and acceleration
class Move_cmd:
    def __init__(self, positionx, positiony, positionz, roll, pitch, yaw, speed, accel):
        self.positionx = positionx
        self.positiony = positiony
        self.positionz = positionz
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.speed = speed
        self.accel = accel

# Class to represent a gripper status
class Gripper_cmd:
    def __init__(self, position, force=50):
        self.position = position #(0-1000)
        self.force = force #(20-100)

# Client class to send MoveXYZW action goals
class MoveXYZWClient(Node):

    def __init__(self):
        super().__init__('MoveXYZW_client')
        # Create action client to communicate with MoveXYZW action server
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')

        # Wait for action server to be available
        self.get_logger().info('Waiting for MoveXYZW action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveXYZW action server is available.')

    def send_goal(self, move_cmd):
        # Validate speed and acceleration values
        if not (0 < move_cmd.speed <= 0.2):
            self.get_logger().warn(f'speed {move_cmd.speed} is invalid, setting to 0.05')
            move_cmd.speed = 0.05
        if not (0 < move_cmd.accel <= 0.2):
            self.get_logger().warn(f'accel {move_cmd.accel} is invalid, setting to 0.03')
            move_cmd.accel = 0.03

        # Prepare the goal message
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = move_cmd.positionx
        goal_msg.positiony = move_cmd.positiony
        goal_msg.positionz = move_cmd.positionz
        goal_msg.yaw = move_cmd.yaw
        goal_msg.pitch = move_cmd.pitch
        goal_msg.roll = move_cmd.roll
        goal_msg.speed = move_cmd.speed
        goal_msg.accel = move_cmd.accel

        # Log the goal details
        self.get_logger().info(f'Request sent: x={goal_msg.positionx}, y={goal_msg.positiony}, z={goal_msg.positionz}, '
                               f'roll={goal_msg.roll}, pitch={goal_msg.pitch}, yaw={goal_msg.yaw}, speed={goal_msg.speed}, accel={goal_msg.accel}')
        
        # Send the goal asynchronously and wait for the result
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        # Get the result of the goal request
        self._goal_handle = self._send_goal_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        # Handle the result of the action
        result = self._get_result_future.result().result
        if result.result == "MoveXYZW:SUCCESS":
            self.get_logger().info('MoveXYZW ACTION succeeded.')
        else:
            self.get_logger().error(f'MoveXYZW ACTION failed with result: {result.result}')

    # Callback to handle feedback received during the execution of the action
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

# Client class to send gripper commands via a service
class GripperCmdClient(Node):
    def __init__(self):
        super().__init__('gripper_cmd_client')
        # Create service client to communicate with the gripper command service
        self.client = self.create_client(Setangle, '/gripper_cmd_Node/ur_gripper_cmd_server')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        self.get_logger().info('Gripper service server is available.')

    # Method to send a command to the gripper (force and position)
    def send_gripper_command(self, gripper_cmd):
        """Send gripper service request"""
        # Create request object
        request = Setangle.Request()
        request.status = "gripper_cmd"
        request.hand_id = 1
        request.angle = [gripper_cmd.force, gripper_cmd.position]

        # Asynchronously send the service request
        self.future = self.client.call_async(request)
        self.get_logger().info(f"Request sent: status = {request.status}, force = {request.angle[0]}, angle = {request.angle[1]}")

        # Manually poll the future's status and wait for a response
        while not self.future.done():
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait 0.1 second each time
        
        # Process the service response
        try:
            response = self.future.result()
            if response.angle_accepted:
                self.get_logger().info('Gripper command accepted successfully.')
            else:
                self.get_logger().error('Gripper command was rejected.')
            return response.angle_accepted
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False

def process_target_in_base(tf_to_depth_camera, camera_data):
    """
    处理并获取目标相对于基坐标系的位置。

    Args:
        tf_to_depth_camera: TF转换模块，用于获取相机的TF信息。
        visualmodule: 视觉处理模块，输入相机数据和相机到基坐标系的变换，输出目标到基坐标系的变换。
        camera_data: 相机捕获的数据。

    Returns:
        target_to_base: 目标相对于基坐标系的变换矩阵（如果成功获取）；否则为 None。
    """
    while rclpy.ok():
        # 获取相机的TF信息
        camera_P_in_base, camera_R_in_base, camera_Euler_in_base, camera_T_in_base = process_transformation(
            tf_to_depth_camera.transformation
        )

        # 如果TF未获取或数据不完整，继续等待
        if camera_T_in_base is None:
            time.sleep(0.1)
            continue

        # 处理相机数据，获取目标变换
        target_to_base = visualmodule(camera_data, camera_T_in_base)
        if target_to_base is not None:
            print("Target to base:", target_to_base)
            cv2.destroyAllWindows()
            return target_to_base

        # 每次循环延时
        time.sleep(0.2)

    # 如果循环结束仍未获取目标，返回 None
    cv2.destroyAllWindows()
    return None


def main(args=None):
    rclpy.init(args=args)
    robot_move_node = MoveXYZWClient()
    gripper_cmd_node = GripperCmdClient()
    transform_node = Node("tf_listener_node")
    visual_node = Node('ggcnn_service_node')

    tf_to_depth_camera = FrameTransformListener(transform_node, "camera_depth_optical_frame", "base_link", timer_period=0.05)
    tf_to_grasp_point = FrameTransformListener(transform_node, "grasp_point", "base_link", timer_period=0.05)
    tf_camera_to_grasp = FrameTransformListener(transform_node, "camera_depth_optical_frame", "grasp_point", timer_period=0.05)
    camera_data = GGCNNService(visual_node, "/kinova/camera")

    # 使用多线程执行器来确保后台spin一个节点或多个节点
    executor = MultiThreadedExecutor()
    executor.add_node(transform_node)
    executor.add_node(visual_node)

    # 在后台线程中spin，使得节点的回调函数不断执行，即使主线程在等待
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # 等待tf_to_grasp_point有数据
    while (tf_to_grasp_point.transformation and tf_camera_to_grasp.transformation) is None and rclpy.ok():
        time.sleep(0.1)  # 等待回调更新

    if (tf_to_grasp_point.transformation and tf_camera_to_grasp.transformation) is not None:
        initial_grasp_P_in_base, initial_grasp_R_in_base, initial_grasp_Euler_in_base, initial_grasp_T_in_base = process_transformation(tf_to_grasp_point.transformation)
        camera_to_grasp, _, _, _ = process_transformation(tf_camera_to_grasp.transformation)
        camera_to_grasp_in_base = initial_grasp_R_in_base @ camera_to_grasp
        print("Initial grasp_Euler_in_base:", initial_grasp_Euler_in_base)
        print("Initial camera_to_grasp:", camera_to_grasp_in_base)
    else:
        print("No initial grasp transformation available.")
        
    vel = 0.2
    acc = 0.2
    target_offset_z = 0.004
    target_offset_x = -0.026
    target_offset_y = 0.012
    gripper_open= Gripper_cmd(1000)
    gripper_grab = Gripper_cmd(800)
    # gripper_grab = Gripper_cmd(700)

    try:
        target_to_base = process_target_in_base(tf_to_depth_camera, camera_data)
        # Define the poses for robot movement
        move_cmd1 = Move_cmd(target_to_base[0] - camera_to_grasp_in_base[0], target_to_base[1] - camera_to_grasp_in_base[1], initial_grasp_P_in_base[2], initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)
        robot_move_node.send_goal(move_cmd1)

        target_to_base = process_target_in_base(tf_to_depth_camera, camera_data)

        move_cmd1 = Move_cmd(target_to_base[0]+target_offset_x, target_to_base[1]+target_offset_y, target_to_base[2]+target_offset_z, initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)
        move_cmd2 = Move_cmd(target_to_base[0], target_to_base[1], target_to_base[2]+0.2, initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)
        move_cmd3 = Move_cmd(target_to_base[0]+0.1, target_to_base[1]+0.1, target_to_base[2]+0.2, initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)
        move_cmd4 = Move_cmd(target_to_base[0]+0.1, target_to_base[1]+0.1, target_to_base[2]+target_offset_z, initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)
        move_cmd5 = Move_cmd(initial_grasp_P_in_base[0], initial_grasp_P_in_base[1], initial_grasp_P_in_base[2], initial_grasp_Euler_in_base[0], initial_grasp_Euler_in_base[1], initial_grasp_Euler_in_base[2], vel, acc)

        gripper_cmd_node.send_gripper_command(gripper_open)
        robot_move_node.send_goal(move_cmd1)
        time.sleep(1)
        gripper_cmd_node.send_gripper_command(gripper_grab)
        time.sleep(1)
        robot_move_node.send_goal(move_cmd2)
        robot_move_node.send_goal(move_cmd3)
        robot_move_node.send_goal(move_cmd4)
        time.sleep(1)
        gripper_cmd_node.send_gripper_command(gripper_open)
        time.sleep(1)
        robot_move_node.send_goal(move_cmd5)


    except KeyboardInterrupt:
        transform_node.get_logger().info("Shutting down nodes.")
    finally:
        executor.shutdown()
        spin_thread.join()
        cv2.destroyAllWindows()
        transform_node.destroy_node()
        visual_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()