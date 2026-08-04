uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_MAPPING=2
uint8 STATE_NAVIGATING=3
uint8 STATE_PATROLLING=4
uint8 STATE_CHARGING=5
uint8 STATE_ERROR=6

string robot_id
builtin_interfaces/Time stamp
uint8 state
float32 battery_percentage
string current_mission