# Orca3 messages

## Messages

| Message | Description | Publisher(s) | Subscriber(s) |
|-----|--------|------|-----|
| Barometer | Barometer reading | driver_node | depth_node |
| Depth | Depth reading | depth_node | base_controller |
| Status | Driver status, includes battery voltage and leak status | driver_node | |
| Thrusters | Thruster pwm values | base_controller | driver_node |
