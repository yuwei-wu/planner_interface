# === Identification ===
int32 drone_id          # Drone Unique ID
int32 traj_id           # Trajectory Unique ID

# === Timing ===
builtin_interfaces/Time start_time
float64 duration

# === Trajectory Type Enum ===
int8 TRAJ_POLY = 0      # standard polynomial
int8 TRAJ_SPLINE = 1   # Bspline or Bernstein
int8 TRAJ_DISCRETE = 2  # discrete method
int8 TRAJ_BD_DERIV = 3  # hermite quintic 

int8 traj_type          # Active trajectory type

# === Union: Only 1 Active Trajectory ===
PiecewisePoly polytraj
Spline splinetraj
Discrete discretetraj
BdDervi bddervitraj