# Drive mode selection published by the mode controller (e.g. keyboard_node).
# drive_mux forwards commands from the matching source to /roomba/drive_command.

uint8 MANUAL        = 0  # keyboard / joystick teleoperation
uint8 WALL_FOLLOW   = 1  # autonomous wall-following (planner_node)
uint8 FOLLOW_ROOMBA = 2  # autonomous Roomba-following (future)

uint8 mode
