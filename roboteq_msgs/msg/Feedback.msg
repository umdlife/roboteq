# 100Hz feedback message for controls purposes
Header header

# Commanded and measured speed of the motors (rad/s)
# Position is reported in rads, and wraps around +-6M
float32 cmd_user
float32 cmd_motor
float32 measured_vel
float32 measured_pos
