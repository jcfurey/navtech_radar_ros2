# One assembled radar revolution with all per-spoke metadata retained.

# Sensor timestamp of the first received spoke and radar frame.
std_msgs/Header header
builtin_interfaces/Time end_stamp

uint32 configuration_version
uint32 sweep_sequence
uint8 sample_bit_depth
float64 range_resolution

# mono16 for a high-precision stream. Rows are increasing encoder azimuth and
# columns are range bins.
sensor_msgs/Image image

# One entry per image row.
uint16[] encoder_values
uint16[] sweep_counters
builtin_interfaces/Time[] timestamps
builtin_interfaces/Time[] host_stamps
