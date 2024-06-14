ros2 service call /hyy_hand_controller/Setangle hyy_message/srv/Setangle "{
    angle0: 1000,
    angle1: 1000,
    angle2: 1000,
    angle3: 1000,
    angle4: 100,
    angle5: 100,
    hand_id: 1, 
    status: 'set_angle'
}"

