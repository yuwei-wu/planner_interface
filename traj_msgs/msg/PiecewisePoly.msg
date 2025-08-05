# Polynomial coefficients for each axis and each polynomial segment 

# Which polynomial basis the coefficients correspond to
uint8 basis_type  # 0=STANDARD, 1=CHEBYSHEV, 2=LEGENDRE, 3=HERMITE, 4=LAGUERRE

# Polynomial coefficients for each axis (x, y, z) and yaw
# ######################## polynomial seting
Polynomial[] seg_x
Polynomial[] seg_y
Polynomial[] seg_z
Polynomial[] seg_yaw