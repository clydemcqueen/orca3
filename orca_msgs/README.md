# Orca3 messages

| Message | Description | Publisher(s) | Subscriber(s) |
|-----|--------|------|-----|
| Armed | Arm or disarm the controller | TeleopNode | BaseController |
| Barometer | Pressure and temperature | OrcaBarometerPlugin, BarometerNode | BaseController |
| CameraTilt | BlueROV2 camera tilt | TeleopNode | DriverNode |
| Depth | ENU depth in meters (+ is above the surface, - is below the surface) | BaseController | |
| Effort | Force and torque divided by maximum force and torque (range \[-1, 1\]) |
| Lights | BlueROV2 lights on / off | TeleopNode | DriverNode |
| Motion | Output of motion model | BaseController | |
| Pid | Output of PID controller | BaseController | |
| Status | Hardware status | OrcaThrusterPlugin, DriverNode | |
| Thrust | Thruster pwm values | BaseController | OrcaThrusterPlugin, DriverNode |
