# Polynomial coefficients for each axis and each polynomial segment 

# Basis type enum
uint8 STANDARD = 0
uint8 CHEBYSHEV = 1
uint8 LEGENDRE = 2
uint8 HERMITE = 3
uint8 LAGUERRE = 4

# Which polynomial basis the coefficients correspond to
uint8 basis_type  

# Polynomial coefficients for each axis (x, y, z) and yaw
# ######################## polynomial seting
Polynomial[] seg_x
Polynomial[] seg_y
Polynomial[] seg_z
Polynomial[] seg_yaw