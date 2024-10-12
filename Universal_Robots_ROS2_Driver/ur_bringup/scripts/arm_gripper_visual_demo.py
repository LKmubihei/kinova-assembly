#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import ACTIONS and SERVICES
from hyy_message.action import MoveXYZW
from hyy_message.srv import Setangle  # Import service message type

import pyrealsense2 as rs
import numpy as np
import cv2

# Import TF2 for transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
import time

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
def visualmodule(base_to_flange):
    """Initialize the camera stream, detect circles, and calculate their positions in the base frame."""
    # Camera intrinsic parameters for ur10e
    K = np.array([[911.1848, 0.0, 646.01928],
                  [0.0, 910.3735, 358.92736],
                  [0.0, 0.0, 1.0]], dtype=np.float64)

    # Transformation matrix from camera to flange
    camera_to_flange = np.array([
        [8.924760722552549375e-03, -0.9995591282682333434, 0.02831779902631070475, 139.5583552397174287 / 1000],
        [0.9999530434162445802, 0.008814175836407645015, -0.004027563396773840843, -32.51499601839394415 / 1000],
        [0.003776189698006349876, 0.02835241435882007027, 0.9995908567966148572, 15.79573133794057505 / 1000],
        [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

    # Configure Realsense stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start the camera stream
    pipeline.start(config)

    # Create an align object to align depth and color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

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
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert color image to grayscale
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to reduce noise
            gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

            # Use Hough Circle Transform to detect circles
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

            # If circles are detected
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # Draw the outer circle
                    cv2.circle(color_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(color_image, (i[0], i[1]), 2, (0, 0, 255), 3)
                    # Print the center coordinates of the circle
                    circle_x = i[0]
                    circle_y = i[1]
                    print("Circle center coordinates: ", circle_x, circle_y)
                    # Get the depth value of the circle center
                    depth_value = depth_frame.get_distance(i[0], i[1])
                    print('Depth value: ', depth_value)
                    print(f"Circle depth value: {depth_value} meters")
                    # Calculate the object's position in the base frame
                    pixel_cam = pixel_to_camera(circle_x, circle_y, K, depth_value)
                    target_to_base = base_to_flange @ camera_to_flange @ pixel_cam
                    print('base_to_flange:', base_to_flange)
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
        pipeline.stop()
        cv2.destroyAllWindows()
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
        self.client = self.create_client(Setangle, '/gripper_cmd_node/ur_gripper_cmd_server')

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

def main(args=None):

    rclpy.init(args=args)

    # Initialize action and service clients
    robot_move_node = MoveXYZWClient()
    gripper_cmd_node = GripperCmdClient()
    
    tf_listener_node = rclpy.create_node('tf_listener_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_listener_node)
    
    # Retrieve the transformation of flange in ur real base
    from_frame = 'tool0'
    to_frame = 'ur_real_base'
    base_to_flange = tf_transformation(tf_buffer, from_frame, to_frame, tf_listener_node)

    if base_to_flange is None:
        tf_listener_node.get_logger().info('Failed to retrieve transformation')
        tf_listener_node.destroy_node()
        rclpy.shutdown()
        return
    
    # Position of flange in the ur real base frame
    P_flange_world = np.array([base_to_flange.transform.translation.x,
                            base_to_flange.transform.translation.y,
                            base_to_flange.transform.translation.z])
    
    # Rotation matrix of the flange in the ur real base frame
    R_flange_world = R.from_quat([
        base_to_flange.transform.rotation.x,
        base_to_flange.transform.rotation.y,
        base_to_flange.transform.rotation.z,
        base_to_flange.transform.rotation.w
    ]).as_matrix()
    
    # Euler angles in the ur real base frame
    rotation_ = R.from_matrix(R_flange_world)
    euler_in_base = rotation_.as_euler('xyz', degrees = False)
    
    # Transformation matrix from camera to world frame
    T_flange_base = np.hstack((R_flange_world, P_flange_world.reshape(-1,1)))
    T_flange_base = np.vstack((T_flange_base, np.array([0, 0, 0, 1])))

    # Call the camera initialization and circle detection function
    target_to_base = visualmodule(T_flange_base)

    if target_to_base is not None:
        print("Target position in base frame from camera:", target_to_base)
    else:
        print("Failed to get target position from camera")
    

    # Define the poses for robot movement
    move_cmd1 = Move_cmd(P_flange_world[0], P_flange_world[1], P_flange_world[2], euler_in_base[0], euler_in_base[1], euler_in_base[2], 0.1, 0.2)
    move_cmd2 = Move_cmd(target_to_base[0], target_to_base[1], target_to_base[2] + 0.2, euler_in_base[0], euler_in_base[1], euler_in_base[2], 0.1, 0.2)
    move_cmd3 = Move_cmd(P_flange_world[0], P_flange_world[1], P_flange_world[2], euler_in_base[0], euler_in_base[1], euler_in_base[2], 0.1, 0.2)
    move_cmd4 = Move_cmd(target_to_base[0], target_to_base[1], target_to_base[2] + 0.2, euler_in_base[0], euler_in_base[1], euler_in_base[2], 0.1, 0.2)
    move_cmd5 = Move_cmd(P_flange_world[0], P_flange_world[1], P_flange_world[2], euler_in_base[0], euler_in_base[1], euler_in_base[2], 0.1, 0.2)

    gripper_open= Gripper_cmd(50, 1000)
    gripper_grab = Gripper_cmd(50, 730)

    #*******************#
    #    action line    #
    #*******************#

    robot_move_node.send_goal(move_cmd1)
    gripper_cmd_node.send_gripper_command(gripper_open)
    robot_move_node.send_goal(move_cmd2)
    time.sleep(2)
    gripper_cmd_node.send_gripper_command(gripper_grab)
    time.sleep(2)
    robot_move_node.send_goal(move_cmd3)
    robot_move_node.send_goal(move_cmd4)
    gripper_cmd_node.send_gripper_command(gripper_open)
    time.sleep(2)
    robot_move_node.send_goal(move_cmd5)
    
    #**********************#
    #    action line end   #
    #**********************#

    # Cleanup nodes and shutdown ROS
    robot_move_node.destroy_node()
    gripper_cmd_node.destroy_node()
    tf_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
