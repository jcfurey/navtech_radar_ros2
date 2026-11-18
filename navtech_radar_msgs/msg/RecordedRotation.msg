# One rotation in acquisition order, transported as a batch during file playback.
# The configuration travels with the samples so the first and last frames, partial
# rotations and discontinuous dataset scenes can be processed independently.
RadarConfiguration configuration
sensor_msgs/Image image
string source_format
uint16[] encoder_values
# Empty if the file contains no packet counters.
uint16[] sweep_counters
# Unix sensor nanoseconds. Empty for recordings with only a frame timestamp.
# Zero means the sensor clock was unset. receipt_ns may then supply recorder time.
uint64[] sensor_ns
uint64[] receipt_ns
# Original per-row validity or chirp byte, when present, including invalid rows.
uint8[] row_flags
