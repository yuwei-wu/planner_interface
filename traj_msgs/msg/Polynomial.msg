# Polynomial coefficients for each axis and each polynomial segment

# Time offsets (durations) from start_time for each sample point
float32[] durations  

# Polynomial order (degree)
uint8 order

# Basis type enum
uint8 STANDARD = 0
uint8 CHEBYSHEV = 1
uint8 LEGENDRE = 2
uint8 HERMITE = 3
uint8 LAGUERRE = 4

# Which polynomial basis the coefficients correspond to
uint8 basis_type  

# Polynomial coefficients for each axis (x, y, z) and yaw
# The coefficients are arranged in order of polynomial degree, e.g., coef_x[0] = a0, coef_x[1] = a1, ...
float32[] coef_x  
float32[] coef_y
float32[] coef_z
float32[] coef_yaw
