# Full Discrete State-Control Trajectory for Quadrotor Dynamics

float64 dt  # Discrete sampling time

# === States ===
geometry_msgs/Point[] pos_pts      # x, y, z positions
geometry_msgs/Point[] vel_pts     # vx, vy, vz velocities
geometry_msgs/Point[] acc_pts  # ax, ay, az accelerations
geometry_msgs/Quaternion[] ori_pts  # Orientation quaternions (optional but preferred for 3D rotations)
geometry_msgs/Point[] angular_vel_pts  # Body angular rates (roll_rate, pitch_rate, yaw_rate)

# === Controls ===
float64[] collective_thrusts           # Collective thrust per time step
geometry_msgs/Point[] moment
geometry_msgs/Point[] body_rate_cmds  # Commanded body rates (roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd)

# Optional: If modeling individual rotor thrusts
# float64[][ ] rotor_thrusts            # 2D array: [time_step][rotor_index]

