# Header
std_msgs/Header header

# Voltage divider
float64 voltage
bool low_battery

# Leak sensor
bool leak_detected

# CPU temp
int32 cpu_temp

# Overall status
uint8 STATUS_NONE=0
uint8 STATUS_OK=1
uint8 STATUS_ABORT=2
uint8 status
