# === Identification ===
int32 drone_id          # Drone Unique ID
int32 traj_id           # Trajectory Unique ID

# === Timing ===
builtin_interfaces/Time start_time
float64 duration

# === Trajectory Type Enum ===
int8 TRAJ_POLY = 0
int8 TRAJ_BSPLINE = 1
int8 TRAJ_DISCRETE = 2
int8 TRAJ_BD_DERIV = 3

int8 traj_type          # Active trajectory type

# === Union: Only 1 Active Trajectory ===
PiecewisePoly polytraj
Bspline bsplinetraj
Discrete discretetraj
BdDervi bddervitraj