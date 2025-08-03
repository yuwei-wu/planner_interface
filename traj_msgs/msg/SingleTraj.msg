# === Identification Info ===
int32 drone_id                    # Unique ID of the drone
int32 traj_id                     # Unique ID of the trajectory instance
builtin_interfaces/Time start_time                # Trajectory start time (seconds, relative or absolute)
builtin_interfaces/Time end_time                  # Trajectory end time (seconds, relative or absolute)

# === Trajectory Type ===
int32 TRAJ_POLYNOMIAL = 0
int32 TRAJ_BSPLINE = 1
int32 TRAJ_DISCRETE = 2
int32 TRAJ_BD_DERIVATIVE = 3

int32 traj_type                   # Specifies which trajectory type is active in this message

# === Trajectory Representations ===
Polynomial polytraj               # Polynomial parameterization (active if traj_type == TRAJ_POLYNOMIAL)
Bspline bsplinetraj               # B-spline parameterization (active if traj_type == TRAJ_BSPLINE)
Discrete discretetraj             # Discrete state-control trajectory (active if traj_type == TRAJ_DISCRETE)
BdDervi bddervitraj               # Boundary Derivatives (active if traj_type == TRAJ_BD_DERIVATIVE)