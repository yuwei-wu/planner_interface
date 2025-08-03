# Bspline.msg — Supports Uniform and Non-uniform B-splines

int32 degree                # Degree of the B-spline (e.g., 3 for cubic)

bool is_uniform              # True if uniform B-spline, False if non-uniform (requires knots)

geometry_msgs/Point[] control_points  # Control points defining the spline curve (x, y, z)

float64[] knots              # Knot vector (only used if is_uniform == False)
                             # Must satisfy standard B-spline knot properties if provided

# Optional: For open uniform B-splines, you can auto-generate the knot vector using degree and control_points.size()
