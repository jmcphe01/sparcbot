# Trailer articulation angle, published by trailer_sensor (/trailer_angle).

std_msgs/Header header
float64 angle_rad            # trailer angle relative to the robot, in [-pi, pi); 0 = straight, + = trailer swung counter-clockwise (seen from above)
uint16 raw                   # raw 12-bit sensor reading, 0..4095 (before zero offset and direction)
bool magnet_ok               # magnet detected and neither too weak nor too strong; if false, angle_rad is not reliable
bool magnet_detected
bool magnet_too_weak         # magnet too far away
bool magnet_too_strong       # magnet too close
