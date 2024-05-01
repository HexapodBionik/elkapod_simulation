# Elkapod Simulation Repository
## Simulation
### Communication with `/elkapod_comm_server` mockup
```bash
ros2 topic pub --once /elkapod_comm_server_leg_frames elkapod_msgs/msg/LegFrames "{
    leg_frames: [
        {leg_nb: 1, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]},
        {leg_nb: 2, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]},
        {leg_nb: 3, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]},
        {leg_nb: 4, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]},
        {leg_nb: 5, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]},
        {leg_nb: 6, servo_op_codes: [1, 1, 1], servo_angles: [0.0, 45.0, -120.0]}
    ]
}" 
```