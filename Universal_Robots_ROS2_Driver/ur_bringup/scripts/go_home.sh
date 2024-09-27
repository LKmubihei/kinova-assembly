ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [
      { 
        positions: [1.733, -1.674, -1.575, -1.464, 1.576, 4.875], time_from_start: { sec: 3, nanosec: 0 },
      }
    ]
  }
}" --feedback