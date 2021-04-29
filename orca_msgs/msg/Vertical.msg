# Deprecated, use Motion.msg

std_msgs/Header header

float64 accel_plan    # Accel to move from v0 to v1
float64 accel_drag    # Accel to counteract drag
float64 accel_hover   # Accel to counteract buoyancy
float64 accel_pid     # Output of PID controller
float64 accel_total

float64 force
float64 effort  # Force normalized to [-1, 1]
