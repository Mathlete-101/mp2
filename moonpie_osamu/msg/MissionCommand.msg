# Message for mission commands
string command  # Command to execute (e.g., "START", "STOP")
int32 dig_time
int32 travel_time 
float32 drive_and_dig_speed_tenths # Drive & dig speed in 0.1 m/s units (e.g. 7 = 0.7 m/s)
float32 backward_travel_speed_tenths # Backward travel speed in 0.1 m/s units (e.g. 7 = 0.7 m/s)
int32 forward_move_time # Time to move forward in 0.1s units
int32 actuator_lower_time # Time to lower actuator in 0.1s units 