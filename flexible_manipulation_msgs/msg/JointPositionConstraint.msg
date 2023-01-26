# Joint limits for IK that shouldn't be exceeded
# Per convention:
# -pi to pi are valid constraints
# -100.0 means no constraint (use URDF joint limits)
# +100.0 means joint locked to current position

# Joint index in SRDF model
uint8 joint_index

# Lower and upper limits
float32 joint_min
float32 joint_max
