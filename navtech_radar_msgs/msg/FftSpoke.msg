# One lossless azimuth packet from a Navtech FFT stream.

# Converted sensor timestamp and radar frame.
std_msgs/Header header

# Local receipt time. During bag adaptation this is simulated delivery time.
builtin_interfaces/Time host_stamp

uint32 configuration_version

# Original Colossus timestamp fields. Legacy bags expose the converted values
# through header.stamp, so the adapter reconstructs these fields from it.
uint32 ntp_seconds
uint32 ntp_split_seconds

uint16 sweep_counter

# Raw encoder count and derived angle in radians. No mount offset is applied.
uint16 azimuth
float64 azimuth_angle

uint8 sample_bit_depth
uint16[] data
