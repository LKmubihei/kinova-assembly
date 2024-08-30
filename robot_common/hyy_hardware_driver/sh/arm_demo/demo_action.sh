ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
    points: [
      { 
        positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 2, nanosec: 0 },
      },
      { 
        positions: [0, 1, 0, -1, 0, -1, 0], time_from_start: { sec: 4, nanosec: 0 },
      },
      {
        positions: [0, -1, 0, 1, 0, 1, 0], time_from_start: { sec: 8, nanosec: 0 },
      },
      { 
        positions: [2, 1, 0, -1, 0, -1, 0], time_from_start: { sec: 12, nanosec: 0 },
      },
      { 
        positions: [-2, 1, 0, -1, 0, -1, 0], time_from_start: { sec: 16, nanosec: 0 },
      },
      { 
        positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 18, nanosec: 0 },
      },
    ]
  }
}" --feedback
