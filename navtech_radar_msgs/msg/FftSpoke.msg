# One lossless azimuth packet from a Navtech FFT stream.

# Converted sensor timestamp and radar frame.
std_msgs/Header header

# Local receipt time. During bag adaptation this is simulated delivery time.
builtin_interfaces/Time host_stamp

# Zero means no configuration was available at receipt time.
uint32 configuration_version

# Original Colossus timestamp fields. Legacy bags expose the converted values
# through header.stamp. The legacy adapter copies seconds and nanoseconds;
# it cannot recover original wire bytes or a different firmware time encoding.
uint32 ntp_seconds
uint32 ntp_split_seconds

uint16 sweep_counter

# Native encoder count and clockwise angle in radians, normalized to [0, 2*pi).
# The derived ROS bearing is yaw offset minus this angle.
uint16 azimuth
float64 azimuth_angle

uint8 sample_bit_depth
uint16[] data
