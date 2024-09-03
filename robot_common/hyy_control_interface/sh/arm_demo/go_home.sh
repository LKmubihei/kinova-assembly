ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [L-J1, L-J2, L-J3, L-J4, L-J5, L-J6, L-J7],
    points: [
      { 
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 3, nanosec: 500 },
      }
    ]
  }
}" --feedback
