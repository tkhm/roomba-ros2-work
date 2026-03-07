# Aggregated sensor data from Roomba Open Interface (OI) sensor packets.

# Bumpers and wheel drops (OI packet 7)
bool bump_left
bool bump_right
bool wheel_drop_left
bool wheel_drop_right

# Cliff sensors (OI packet 9)
bool cliff_left
bool cliff_front_left
bool cliff_front_right
bool cliff_right

# Odometry — delta since last read (OI packets 19, 20)
int16 distance_mm     # distance traveled since last sensor read [mm]
int16 angle_degrees   # angle turned since last sensor read [degrees]

# Battery (OI packets 25, 26)
uint16 battery_voltage_mv  # battery voltage [mV]
int16  battery_current_ma  # battery current [mA] (negative = discharging)
