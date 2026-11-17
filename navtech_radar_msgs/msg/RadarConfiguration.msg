# Configuration associated with a Navtech FFT stream.

# Receipt time and radar frame. Configuration packets do not carry sensor time.
std_msgs/Header header

# Incremented whenever any configuration field changes.
uint32 configuration_version

uint16 azimuth_samples
uint16 encoder_size

# Exact Colossus field. One unit is 0.0001 metres for the CTS350-X stream.
uint16 bin_size
uint16 range_bins

# Nominal SI spacing in metres per bin before range calibration.
float64 range_resolution

# Revolutions and spokes per second, respectively.
uint16 expected_rotation_rate
uint16 packet_rate

# Metres at bin i = i * range_resolution * range_gain + range_offset.
# Gain is dimensionless, offset is in metres. Use (1, 0) for nominal geometry.
# For compatibility a zero-initialized pair (0, 0) also means nominal geometry.
float32 range_gain
float32 range_offset
uint8 sample_bit_depth
