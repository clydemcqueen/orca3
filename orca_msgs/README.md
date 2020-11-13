# Orca3 messages

| Message | Description | Publisher(s) | Subscriber(s) |
|-----|--------|------|-----|
| Barometer | Pressure and temperature | OrcaBarometerPlugin | DepthNode |
| Depth | ENU depth in meters (+ is above the surface, - is below the surface) | DepthNode | BaseController |
| Effort | Force and torque divided by maximum force and torque (range \[-1, 1\]) |
| Status | Hardware status | OrcaThrusterPlugin | rov_node, auv_node |
| Thrust | Thruster pwm values | BaseController | OrcaThrusterPlugin |
