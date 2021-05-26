std_msgs/Header header

float64 dt

# Pose is in the world frame; acceleration, velocity, force and effort are in the robot frame

geometry_msgs/Accel accel_model   # Accel to move from v0 to v1
geometry_msgs/Accel accel_drag    # Accel to counteract drag
geometry_msgs/Accel accel_hover   # Accel to counteract buoyancy
geometry_msgs/Accel accel_pid     # Output of PID controller(s)
geometry_msgs/Accel accel_total

geometry_msgs/Twist cmd_vel       # Input to motion model; get a nice timestamp by having this here
geometry_msgs/Twist vel           # Output from motion model

geometry_msgs/Pose pose

geometry_msgs/Wrench force
orca_msgs/Effort effort  # Force normalized to [-1, 1]
