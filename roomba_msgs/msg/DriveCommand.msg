# Drive command for Roomba differential drive.
# Each wheel velocity is clamped to [-500, 500] mm/s by roomba_node.
int16 left_mm_s   # left wheel velocity [mm/s]  (positive = forward)
int16 right_mm_s  # right wheel velocity [mm/s] (positive = forward)
