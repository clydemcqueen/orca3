std_msgs/Header header

float64 thrust_msg_lag
float64 voltage

uint32 STATUS_NONE=0                  # Status not available (e.g., driver crashed)
uint32 STATUS_READY=200               # Not receiving /thrust messages
uint32 STATUS_RUNNING=201             # Receving /thrust messages
uint32 STATUS_ABORT_HARDWARE=500      # Abort dive: hardware failure
uint32 STATUS_ABORT_LOW_BATTERY=501   # Abort dive: low battery
uint32 STATUS_ABORT_LEAK=502          # Abort dive: leak detected
uint32 status
