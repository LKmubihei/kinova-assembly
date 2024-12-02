#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Import ACTIONS and SERVICES
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle  # Import service message type

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

# Import TF2 for transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
import time
from ultralytics import YOLO
from sensor_msgs.msg import Image,CameraInfo

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

# Function to retrieve transformation between two frames
def tf_transformation(tf_buffer, from_frame, to_frame, node):
    """Retrieve the transformation between two frames."""
    time = rclpy.time.Time()
    count = 0
    transformation = None
    received_transformation = False

    # Try to retrieve the transformation up to 100 attempts
    while not received_transformation and count < 100:
        try:
            transformation = tf_buffer.lookup_transform(to_frame, from_frame, time)
            node.get_logger().info(f'Received transformation: {transformation}')
            received_transformation = True
        except TransformException:
            node.get_logger().info('Error: Transformation not received')
            rclpy.spin_once(node, timeout_sec=1.0)
            count += 1
            continue
    return transformation

# Visual module to handle camera input and circle detection
def visualmodule(camera_to_flange, model, phase ,rgb, depth, K):
    """Initialize the camera stream, detect circles, and calculate their positions in the base frame."""
    print(f"phase is {phase}!!!!")
    # Camera intrinsic parameters for ur10e
    # K = np.array([[911.1848, 0.0, 646.01928],
    #               [0.0, 910.3735, 358.92736],
    #               [0.0, 0.0, 1.0]], dtype=np.float64)

    # Transformation matrix from camera to flange
    # camera_to_flange = np.array([
    #     [8.924760722552549375e-03, -0.9995591282682333434, 0.02831779902631070475, 139.5583552397174287 / 1000],
    #     [0.9999530434162445802, 0.008814175836407645015, -0.004027563396773840843, -32.51499601839394415 / 1000],
    #     [0.003776189698006349876, 0.02835241435882007027, 0.9995908567966148572, 15.79573133794057505 / 1000],
    #     [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

    # Configure Realsense stream
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # # Start the camera stream
    # pipeline.start(config)

    # # Create an align object to align depth and color frames
    # align_to = rs.stream.color
    # align = rs.align(align_to)

    target_to_base = None  # Initialize return value

    try:
        while True:
            # Get frames from the camera
            frames = pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)

            # Retrieve aligned color and depth frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("No depth or color frame received")
                continue

            # Convert color and depth frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            # cv2.rectangle(color_image, (10, 10), (100, 100), (0, 255, 0), 2)
            depth_image = np.asanyarray(depth_frame.get_data())

            results = model(color_image, imgsz=640, conf=0.3)

            boxes = results[0].boxes.xywh.cpu().numpy()
            # print(boxes)
            # print(len(boxes))
            boxes_xyxy = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()

            if phase == 1 or phase == 2:  ## find redgear
                if len(boxes) > 0:
                    for box, box_xyxy, cls_ in zip(boxes, boxes_xyxy, classes):
                        if int(cls_) == 0:      ## redgear
                            print(box_xyxy[0], box_xyxy[1], box_xyxy[2], box_xyxy[3])
                            # cv2.rectangle(color_image, (box_xyxy[0], box_xyxy[1]), (box_xyxy[2], box_xyxy[3]), (0, 255, 0), 2)
                            circle_x = int(box[0])
                            circle_y = int(box[1])
                            print("object center coordinates: ", circle_x, circle_y)
                            # Draw the center of the circle
                            cv2.circle(color_image, (circle_x, circle_y), 2, (0, 255, 0), 3)
                            # Get the depth value of the circle center
                            depth_value = depth_frame.get_distance(circle_x, circle_y)
                            print('Depth value: ', depth_value)
                            print(f"Circle depth value: {depth_value} meters")
                            # Calculate the object's position in the base frame
                            pixel_cam = pixel_to_camera(circle_x, circle_y, K, depth_value)
                            target_to_base = camera_to_flange @ camera_to_flange @ pixel_cam
                            print('camera_to_flange:', camera_to_flange)
                            print('camera_to_flange:', camera_to_flange)
                            print('pixel_to_camera:', pixel_cam)
                            print("target_to_base:", target_to_base)
                            break  # Exit loop after detecting one circle
            elif phase == 3:  ## find redshaftseat
                if len(boxes) > 0:
                    for box, box_xyxy, cls_ in zip(boxes, boxes_xyxy, classes):
                        if int(cls_) == 1:      ## redshaftseat
                            # cv2.rectangle(color_image, (box_xyxy[0], box_xyxy[1]), (box_xyxy[2], box_xyxy[3]), (0, 255, 0), 2)
                            circle_x = int(box[0]) 
                            circle_y = int(box[1])
                            print("object center coordinates: ", circle_x, circle_y)
                            # Draw the center of the circle
                            cv2.circle(color_image, (circle_x, circle_y), 2, (0, 255, 0), 3)
                            # Get the depth value of the circle center
                            depth_value = depth_frame.get_distance(circle_x, circle_y)
                            print('Depth value: ', depth_value)
                            print(f"Circle depth value: {depth_value} meters")
                            # Calculate the object's position in the base frame
                            pixel_cam = pixel_to_camera(circle_x, circle_y, K, depth_value)
                            target_to_base = camera_to_flange @ camera_to_flange @ pixel_cam
                            print('camera_to_flange:', camera_to_flange)
                            print('camera_to_flange:', camera_to_flange)
                            print('pixel_to_camera:', pixel_cam)
                            print("target_to_base:", target_to_base)
                            break  # Exit loop after detecting one circle
            elif phase == 4:  ## find redcircle
                print(boxes)
                if len(boxes) > 0:
                    for box, box_xyxy, cls_ in zip(boxes, boxes_xyxy, classes):
                        if int(cls_) == 2:      ## redcircle
                            # cv2.rectangle(color_image, (box_xyxy[0], box_xyxy[1]), (box_xyxy[2], box_xyxy[3]), (0, 255, 0), 2)
                            circle_x = int(box[0])
                            circle_y = int(box[1])
                            print("object center coordinates: ", circle_x, circle_y)
                            # Draw the center of the circle
                            cv2.circle(color_image, (circle_x, circle_y), 2, (0, 255, 0), 3)
                            # Get the depth value of the circle center
                            depth_value = depth_frame.get_distance(circle_x, circle_y)
                            print('Depth value: ', depth_value)
                            print(f"Circle depth value: {depth_value} meters")
                            # Calculate the object's position in the base frame
                            pixel_cam = pixel_to_camera(circle_x, circle_y, K, depth_value)
                            target_to_base = camera_to_flange @ camera_to_flange @ pixel_cam
                            print('camera_to_flange:', camera_to_flange)
                            print('camera_to_flange:', camera_to_flange)
                            print('pixel_to_camera:', pixel_cam)
                            print("target_to_base:", target_to_base)
                            break  # Exit loop after detecting one circle
            

            # Display the color image
            cv2.imshow('Color Image', color_image)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the camera stream
        # pipeline.stop()
        cv2.destroyAllWindows()
        
        if target_to_base is not None:
            print("Target position in base frame from camera:", target_to_base)
        else:
            print("Failed to get target position from camera")

        # Return the calculated target_to_base
        return target_to_base

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
    def __init__(self, force, position):
        self.force = force #(20-100)
        self.position = position #(0-1000)

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

def get_flange_pose(tf_buffer, tf_listener_node, from_frame, to_frame):

    # Retrieve the transformation
    camera_to_flange = tf_transformation(tf_buffer, from_frame, to_frame, tf_listener_node)

    if camera_to_flange is None:
        tf_listener_node.get_logger().info('Failed to retrieve transformation')
        return None

    # Extract translation vector
    camera_P_in_base = np.array([
        camera_to_flange.transform.translation.x,
        camera_to_flange.transform.translation.y,
        camera_to_flange.transform.translation.z
    ])

    # Extract rotation as a matrix
    camera_R_in_base = R.from_quat([
        camera_to_flange.transform.rotation.x,
        camera_to_flange.transform.rotation.y,
        camera_to_flange.transform.rotation.z,
        camera_to_flange.transform.rotation.w
    ]).as_matrix()

    # Convert rotation matrix to Euler angles
    camera_Euler_in_base = R.from_matrix(camera_R_in_base).as_euler('xyz', degrees=False)

    # 打印结果
    print("位置(Translation Vector):\n", camera_P_in_base)
    print("旋转矩阵(Rotation Matrix):\n", camera_R_in_base)
    print("欧拉角(Euler Angles):\n", camera_Euler_in_base)

    return camera_P_in_base, camera_R_in_base, camera_Euler_in_base

def create_homogeneous_transform(translation_vector, rotation_matrix):

    # Ensure the translation vector is a column vector
    translation_vector = np.array(translation_vector).reshape(-1, 1)
    
    # Horizontally stack the rotation matrix and translation vector
    T_3x4 = np.hstack((rotation_matrix, translation_vector))
    
    # Vertically stack the bottom row [0, 0, 0, 1] to create a 4x4 matrix
    T_4x4 = np.vstack((T_3x4, np.array([0, 0, 0, 1])))

    return T_4x4

class GGCNNservice(Node):
    def __init__(self):
        super().__init__('ggcnn_service')

        # init image subscribers
        self.rgb_sub = self.create_subscription(Image,'/color/image_raw',self.rgb_callback,1)
        self.depth_sub = self.create_subscription(Image,'/depth/image_raw',self.depth_callback,1)
        # init cam_info subscriber to obtain intrinsic matrix
        self.depth_info_sub = self.create_subscription(CameraInfo,'/depth/camera_info',self.depth_info_callback,1)
        self.get_logger().info('Initialized subscribers')

        # depth camera params
        self.K_depth = []
        self.depth_scale = 1000
        self.h_depth = 0
        self.w_depth = 0
        # to check if msgs are received
        self.received_depth = False
        self.received_rgb = False
        self.received_depth_K = False
        self.received_transformation = False
        # cv bridge to convert imgmsg to cv2img
        self.cv_bridge = CvBridge()

    def rgb_callback(self,img_msg):
        self.rgb = self.cv_bridge.imgmsg_to_cv2(img_msg)
        if not self.received_rgb:
            self.get_logger().info(f'Received RGB image: {img_msg.width}X{img_msg.height}')
        self.received_rgb = True

    def depth_callback(self,img_msg):
        self.depth = self.cv_bridge.imgmsg_to_cv2(img_msg)
        self.h_depth = img_msg.height
        self.w_depth = img_msg.width

        if not self.received_depth:
            self.get_logger().info(f'Received depth image: {self.w_depth}X{self.h_depth}')
        self.received_depth = True
    def depth_info_callback(self,info_msg):
        self.K_depth = info_msg.k
        if not self.received_depth_K:
            self.get_logger().info(f'Received Depth Camera Intrinsic Matrix:\n {self.K_depth}')
        self.received_depth_K = True

    # function to obtain transformation between frames       
    def tf_transformation(self,from_frame,to_frame):
        self.time = rclpy.time.Time()
        count = 0
        transformation = None
        while ( (not self.received_transformation) and (count < 100) ):
            try:
                transformation: TransformStamped = self.tf_buffer.lookup_transform(to_frame, from_frame, self.time)
                self.get_logger().info(f'Received Transform: {transformation}')
                self.received_transformation = True
            except TransformException:
                self.get_logger().info('Error: Unable to receive trasnform')
                time.sleep(1)
                continue
            count += 1
        return transformation

    def show_image(self,img,title):
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype('uint8')
        cv2.imwrite(title, img)
        # cv2.imshow(title, img)
        # cv2.waitKey() 

def main(args=None):

    rclpy.init(args=args)

    # Initialize action and service clients
    robot_move_node = MoveXYZWClient()
    gripper_cmd_node = GripperCmdClient()
    tf_listener_node = rclpy.create_node('tf_listener_node')
    visual_node = GGCNNservice()

    executor = MultiThreadedExecutor()
    executor.add_node(visual_node)

    # 运动到指定位姿
    # move_cmd= Move_cmd(camera_P_in_base[0], camera_P_in_base[1], camera_P_in_base[2], camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2], 0.1, 0.2)
    # robot_move_node.send_goal(move_cmd)

    vel = 0.05
    acc = 0.1
    calibraoffset_x = 0.000
    cameraoffset = -0.14
    gripperoffset = 0.2
    gripper_open= Gripper_cmd(50, 1000)
    gripper_grab = Gripper_cmd(50, 730)
    from_frame = 'd435_depth_frame'
    to_frame = 'base_link'
    model = YOLO('/home/slam/ur_ros2_ws/src/Universal_Robots_ROS2_Driver/ur_bringup/scripts/best.pt')

    #************************#
    #    action line start   #
    #************************#

    # 第一次，识别齿轮位置，移动到齿轮上方，并打开夹爪

    # 获取法兰盘位姿
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_listener_node)
    gripper_cmd_node.send_gripper_command(gripper_open)
    camera_P_in_base, camera_R_in_base, camera_Euler_in_base = get_flange_pose(tf_buffer, tf_listener_node, from_frame, to_frame)
    move_cmd_initial = Move_cmd(camera_P_in_base[0], camera_P_in_base[1], camera_P_in_base[2], camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2], vel, acc)    
    # 生成法兰盘在基座坐标系中的齐次变换矩阵
    flange_Transfor_in_base = create_homogeneous_transform(camera_P_in_base, camera_R_in_base)
    # 调用视觉模块，传入齐次变换矩阵和模型
    target_to_base = visualmodule(flange_Transfor_in_base, model, 1, visual_node.rgb, visual_node.depth, visual_node.K_depth)
    # Define the poses for robot movement
    move_cmd = Move_cmd(target_to_base[0], target_to_base[1] + cameraoffset, camera_P_in_base[2], camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2],vel, acc)    
    robot_move_node.send_goal(move_cmd)

    # 第二次，识别齿轮，进行抓取，并抬起机械臂到识别位置

    # 获取法兰盘位姿
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_listener_node)
    camera_P_in_base, camera_R_in_base, camera_Euler_in_base = get_flange_pose(tf_buffer, tf_listener_node, from_frame, to_frame)
    # 生成法兰盘在基座坐标系中的齐次变换矩阵
    flange_Transfor_in_base = create_homogeneous_transform(camera_P_in_base, camera_R_in_base)
    # 调用视觉模块，传入齐次变换矩阵和模型
    target_to_base = visualmodule(flange_Transfor_in_base, model, 2)
    # Define the poses for robot movement
    move_cmd = Move_cmd(target_to_base[0] + calibraoffset_x, target_to_base[1], target_to_base[2] + gripperoffset, camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2], vel, acc)
    robot_move_node.send_goal(move_cmd)
    time.sleep(2)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    time.sleep(2)
    robot_move_node.send_goal(move_cmd_initial)

    # # 第三次，识别齿轮轴，运动到齿轮轴的上方

    # # 获取法兰盘位姿
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_listener_node)
    camera_P_in_base, camera_R_in_base, camera_Euler_in_base = get_flange_pose(tf_buffer, tf_listener_node, from_frame, to_frame)
    # 生成法兰盘在基座坐标系中的齐次变换矩阵
    flange_Transfor_in_base = create_homogeneous_transform(camera_P_in_base, camera_R_in_base)
    # 调用视觉模块，传入齐次变换矩阵和模型
    target_to_base = visualmodule(flange_Transfor_in_base, model, 3)
    # Define the poses for robot movement
    move_cmd = Move_cmd(target_to_base[0], target_to_base[1] + cameraoffset, camera_P_in_base[2], camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2], vel, acc)
    robot_move_node.send_goal(move_cmd)

    # 第四次，识别齿轮轴，移动齿轮到齿轮轴中, 松开夹爪，完成轴孔装配，并回到初始位置 

    # 获取法兰盘位姿
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_listener_node)
    camera_P_in_base, camera_R_in_base, camera_Euler_in_base = get_flange_pose(tf_buffer, tf_listener_node, from_frame, to_frame)
    # 生成法兰盘在基座坐标系中的齐次变换矩阵
    flange_Transfor_in_base = create_homogeneous_transform(camera_P_in_base, camera_R_in_base)
    # 调用视觉模块，传入齐次变换矩阵和模型
    target_to_base = visualmodule(flange_Transfor_in_base, model, 4)
    # Define the poses for robot movement
    move_cmd = Move_cmd(target_to_base[0] + calibraoffset_x, target_to_base[1], target_to_base[2] + gripperoffset + 0.005, camera_Euler_in_base[0], camera_Euler_in_base[1], camera_Euler_in_base[2], vel, acc)
    robot_move_node.send_goal(move_cmd)
    time.sleep(2)
    gripper_cmd_node.send_gripper_command(gripper_open)
    time.sleep(2)
    robot_move_node.send_goal(move_cmd_initial)
    
    #**********************#
    #    action line end   #
    #**********************#

    # Cleanup nodes and shutdown ROS
    try:
        # 保持节点运行
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        visual_node.destroy_node()
        robot_move_node.destroy_node()
        gripper_cmd_node.destroy_node()
        tf_listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
