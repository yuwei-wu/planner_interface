# Spline.msg — Supports B-splines and Bernstein (Bezier) polynomials

int8 spline_type            # 0: B-spline, 1: Bernstein (Bezier)

int8 order                  # Degree of the spline (e.g., 3 for cubic)

geometry_msgs/Point[] pos_pts  # Control points defining the curve (x, y, z)

# Only relevant for B-splines:
bool is_uniform             # True for uniform B-spline; False for non-uniform (uses knots)
float64[] knots              # Knot vector (only used if spline_type == SPLINE_BSPLINE and is_uniform == False)
                            # Must satisfy standard B-spline knot properties if provided

# For Bernstein (Bezier), is_uniform and knots are ignored.
# For open uniform B-splines, knot vector can be auto-generated using degree and pos_pts.size()