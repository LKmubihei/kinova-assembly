ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [L-J1, L-J2, L-J3, L-J4, L-J5, L-J6, L-J7],
    points: [
      { 
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 3, nanosec: 0 },
      }
    ]
  }
}" --feedback
ros2 action send_goal /right_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [R-J1, R-J2, R-J3, R-J4, R-J5, R-J6, R-J7],
    points: [
      { 
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 3, nanosec: 0 },
      }
    ]
  }
}" --feedback
