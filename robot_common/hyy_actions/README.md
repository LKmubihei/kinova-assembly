## Hyy Ros2 Actions
The Robot Actions/Triggers are independent ROS2 Actions that execute different robot/end-effector motions in Gazebo simulation. The list below explains what every single ROS2 Action does, and how the actions are executed independently using the Ubuntu Terminal:

* MoveJ: The Robot moves to the specific waypoint, which is specified by Joint Pose values.
  ```sh
  ros2 action send_goal -f /MoveJ hyy_message/action/MoveJ "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}" # (6-DOF)
  ros2 action send_goal -f /MoveJs hyy_message/action/MoveJs "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.00}, speed: 1.0}" # (7-DOF)
  ```
* MoveG: The Gripper fingers move to the specific pose.
  ```sh
  ros2 action send_goal -f /MoveG hyy_message/action/MoveG "{goal: 0.00, speed: 1.0}"
  ```
* MoveL: The Robot executes a CARTESIAN/LINEAR path. The End-Effector orientation is kept constant, and the position changes by +-(x,y,z).
  ```sh
  ros2 action send_goal -f /MoveL hyy_message/action/MoveL "{movex: 0.00, movey: 0.00, movez: 0.00, speed: 1.0}"
  ```
* MoveR: The Robot rotates the selected joint a specific amount of degrees.
  ```sh
  ros2 action send_goal -f /MoveR hyy_message/action/MoveR "{joint: '---', value: 0.00, speed: 1.0}"
  ```
* MoveXYZW: The Robot moves to the specific waypoint, which is represented by the Position(x,y,z) + EulerAngles(yaw,pitch,roll) End-Effector coordinates.
  ```sh
  ros2 action send_goal -f /MoveXYZW hyy_message/action/MoveXYZW "{positionx: 0.00, positiony: 0.00, positionz: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00, speed: 1.0}"
  ```
* MoveXYZ: The Robot moves to the specific waypoint -> Position(x,y,z) maintaining the End-Effector orientation.
  ```sh
  ros2 action send_goal -f /MoveXYZ hyy_message/action/MoveXYZ "{positionx: 0.00, positiony: 0.00, positionz: 0.00, speed: 1.0}"
  ```
* MoveYPR: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). The YPR(yaw,pitch,roll)determines the FINAL ROTATION of the End-Effector, which is related to the GLOBAL COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /MoveYPR hyy_message/action/MoveYPR "{yaw: 0.00, pitch: 0.00, roll: 0.00, speed: 1.0}"
  ```
* MoveROT: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /MoveROT hyy_message/action/MoveROT "{yaw: 0.00, pitch: 0.00, roll: 0.00, speed: 1.0}"
  ```
* MoveRP: End-Effector rotation AROUND A POINT -> The Robot rotates/orientates + moves the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll) + Point(x,y,z). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME, AROUND THE (x,y,z) POINT.
  ```sh
  ros2 action send_goal -f /MoveRP hyy_message/action/MoveRP "{yaw: 0.00, pitch: 0.00, roll: 0.00, x: 0.0, y: 0.0, z: 0.0, speed: 1.0}"
  ```
* NOTE: The Robot JOINT SPEED is controlled by the "speed" parameter when executing the specific ROS2.0 action. The value must be (0,1]. being 1 the maximum velocity and 0 the null velocity (which is not valid -> A small value must be defined, e.g.: 0.01 represents a very slow movement).