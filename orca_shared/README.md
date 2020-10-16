# `mw`

The `mw` (message wrapper) library provides core classes for Orca.

### Design

* Largely 1:1 object:msg, though msg is optional
* Message round trip: Class{msg}.msg() -> msg
* Default constructable
* May be immutable
* Message composition leads to object composition
* Stream-printable

### Core classes

| Class | Message | Description |
|-----|-----|-----|
| Accel | geometry_msgs::msg::Accel | Acceleration |
| Efforts |  | Thrust efforts in the range [-1, 1] |
| Header | std_msgs::msg::Header | Time stamp + frame id |
| Point | geometry_msgs::msg::Point | Point |
| Pose | geometry_msgs::msg::Pose | Point + Quaternion |
| PoseStamped | geometry_msgs::msg::PoseStamped | Header + Pose |
| Quaternion | geometry_msgs::msg::Quaternion | Quaternion |
| Twist | geometry_msgs::msg::Twist | Velocity |

### Unwrapped messages

| Message | Description |
|-----|-----|
| orca_msgs::msg::Barometer | Barometer reading |
| orca_msgs::msg::Depth | Depth reading |
| orca_msgs::msg::Driver | Driver status |
| orca_msgs::msg::Thrusters | Thruster pwm values |
