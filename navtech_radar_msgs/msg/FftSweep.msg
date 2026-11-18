# One assembled radar revolution with all per-spoke metadata retained.

# Acquisition time of the first received spoke and radar frame. File playback
# uses the recorder or frame timestamp when the sensor supplied no row time.
std_msgs/Header header
builtin_interfaces/Time end_stamp

uint32 configuration_version
uint32 sweep_sequence
uint8 sample_bit_depth
# Nominal metres per bin. Use this version's RadarConfiguration for gain and offset.
float64 range_resolution

# mono16 for a high-precision stream. Rows are increasing encoder azimuth and
# columns are range bins. Duplicate encoder rows are retained in arrival order;
# missing rows are omitted, not zero-filled. Partial sweeps can have fewer or
# more rows than the configured azimuth_samples. All rows have the same width,
# frame, sample_bit_depth, and configuration_version.
sensor_msgs/Image image

# One entry per image row.
uint16[] encoder_values
uint16[] sweep_counters
builtin_interfaces/Time[] timestamps
builtin_interfaces/Time[] host_stamps
