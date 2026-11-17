# Navtech raw-data driver design

This note records the driver research for the Navtech CTS350-X data in The
Great Outdoors Dataset and the proposed interface for a production ROS 2
driver. It is intentionally centered on preserving the radar data rather than
producing a particular point-cloud representation.

## What the bag contains

The dataset identifies the radar as a Navtech CTS350-X. The converted bag
contains the original high-precision FFT stream on `/Navtech/FFTData` as
`nav_ross/msg/HighPrecisionFFTData`:

- 400 spokes per revolution
- 6,000 range bins per spoke
- 0.0438 m per range bin
- 4 revolutions per second
- 16-bit, unscaled FFT samples
- encoder azimuth, sweep counter, and radar timestamp on every spoke

This is raw FFT/video data, not raw ADC or complex I/Q data. The exact
reconstructed legacy schemas are in [nav_ross/msg](../nav_ross/msg).

The source was almost certainly the high-resolution path in
[`unmannedlab/navtechradar_usl`](https://github.com/unmannedlab/navtechradar_usl),
`ros1/src/talker1.cpp`. Its field names, topic names, NTP timestamps, encoder
values, and `uint16` payload match the bag. It also explains the unusually
large configuration-topic count: it republishes the static configuration for
every spoke. That repository is MIT licensed.

No radar-to-robot extrinsic is published in this bag or offered by the
dataset's calibration downloads. The transform used by this workspace is
therefore only a visualization placeholder.

## Existing driver assessment

### Best protocol and transport reference

[`Unmanned-Systems-Lab/navtech_radar_ros_sdk`](https://github.com/Unmanned-Systems-Lab/navtech_radar_ros_sdk)
contains a 2024 Navtech IASDK and ROS 2 examples. The nested IASDK is MIT
licensed and supports Colossus message type 31, `high_precision_fft_data`.
Both its C++ protocol implementation and Python parser understand the 16-bit
wire payload.

This is the best starting point for the TCP connection, message framing,
configuration parsing, timestamps, and reconnect behavior. Its ROS wrappers
should not define our public API:

- `RadarFftDataMsg` carries only `uint8[] data`.
- B-scan and extended B-scan allocate `uint8` images and publish `8UC1`.
- Processing examples assume 8-bit power values.
- Some example calculations make unsafe encoder/azimuth assumptions.

The extended B-scan's per-row encoder and timestamp arrays are useful design
ideas, but the sample values must remain `uint16` for this sensor mode.

### Other candidates

- [`norlab-ulaval/navtech_radar_ros`](https://github.com/norlab-ulaval/navtech_radar_ros)
  is an archived ROS 2/Humble fork of older Navtech examples. It is useful for
  ROS integration comparison, but has the same 8-bit-oriented processing
  model and older SDK dependency.
- [`biomotion/navtech_projection`](https://github.com/biomotion/navtech_projection)
  is not a hardware driver. It consumes an already assembled `/Navtech/Polar`
  `mono8` image, remaps it to Cartesian coordinates, and thresholds a point
  cloud. It loses the original 16-bit samples and per-spoke metadata.
- [`utiasASRL/steam_icp`](https://github.com/utiasASRL/steam_icp) is a useful
  downstream radar odometry/processing reference, not a Navtech transport
  driver.

## Proposed ROS 2 architecture

Keep transport, ROS publication, and derived processing separable:

1. `navtech_colossus`: a small C++ library around the pinned MIT IASDK
   transport/protocol code.
2. `navtech_radar_msgs`: hardware-independent configuration, spoke, and sweep
   interfaces.
3. `navtech_radar_driver`: a composable lifecycle node that owns the TCP
   connection and publishes exact spokes.
4. `navtech_radar_processing`: optional sweep assembly, visualization,
   projection, CFAR, and point-cloud components.

The hardware driver should not publish a mount transform. A robot description
or calibration package owns that transform.

## Proposed message semantics

The canonical output should be one lossless spoke message per radar packet.
These names are implemented in `navtech_radar_msgs`.

```text
# RadarConfiguration.msg
std_msgs/Header header                 # host receipt time and radar frame
uint32 configuration_version
uint16 azimuth_samples
uint16 encoder_size
uint16 bin_size                        # exact Colossus configuration field
uint16 range_bins
float64 range_resolution               # nominal metres per bin
uint16 expected_rotation_rate          # revolutions per second
uint16 packet_rate                     # spokes per second, when available
float32 range_gain
float32 range_offset
uint8 sample_bit_depth                 # 8 or 16
```

```text
# FftSpoke.msg
std_msgs/Header header                 # converted sensor time and radar frame
builtin_interfaces/Time host_stamp     # local receipt time
uint32 configuration_version
uint32 ntp_seconds                     # exact wire timestamp fields
uint32 ntp_split_seconds
uint16 sweep_counter
uint16 azimuth                         # encoder count, not a row index
float64 azimuth_angle                  # native clockwise radians
uint8 sample_bit_depth
uint16[] data                          # exact high-precision FFT bins
```

Range calibration uses zero-based FFT bin indices. The processing packages
apply the configured gain to the nominal bin spacing and add the offset in
metres. The legacy adapter supplies unit gain and zero offset because its
configuration message omits those fields. [Navtech Colossus configuration](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2061009217)

Keeping both the converted sensor timestamp and original timestamp fields
avoids losing information if firmware variants interpret the split-second
field differently. The host stamp allows latency and clock-offset analysis.

An optional sweep assembler can publish:

```text
# FftSweep.msg
std_msgs/Header header                 # first-spoke sensor time
builtin_interfaces/Time end_stamp
uint32 configuration_version
uint32 sweep_sequence
uint8 sample_bit_depth
float64 range_resolution
sensor_msgs/Image image                # mono16; rows=spokes, columns=bins
uint16[] encoder_values                # one per image row
uint16[] sweep_counters                # one per image row
builtin_interfaces/Time[] timestamps   # one per image row
builtin_interfaces/Time[] host_stamps  # one per image row
```

The assembler should index rows from encoder values and configuration, detect
wraparound, report missing/duplicate spokes, and publish partial sweeps only
under an explicit policy. It should not silently sort away duplicated encoder
values.

## Topics and QoS

Current relative topic defaults (the topic parameters and ROS remapping can
override these):

- `navtech_radar/configuration`: reliable, transient local, keep last 1; publish only when
  the configuration changes.
- `navtech_radar/fft/spoke`: sensor-data QoS, bounded queue; the lossless canonical stream.
- `navtech_radar/fft/sweep`: reliable, volatile, depth 2; optional assembled output.
- `navtech_radar/fft/image`: reliable, volatile, depth 2; the sweep image alone.
- `navtech_radar/point_cloud`: sensor-data QoS; optional lossy peak selection.
- `diagnostics`: standard ROS diagnostics for connection and packet health.

The node should expose configurable topic/frame names rather than retaining
the legacy uppercase global names.

## Driver behavior

The first implementation should include:

- lifecycle configure/activate/deactivate/cleanup behavior;
- configurable address and port (Colossus normally uses TCP port 6317);
- high-precision stream selection and an option to reject unexpected 8-bit
  data rather than silently widening it;
- bounded queues and no allocation proportional to an untrusted packet size;
- reconnect with backoff and clear state reset at reconnection;
- packet, malformed-message, duplicate, dropped-spoke, and incomplete-sweep
  diagnostics;
- explicit sensor-time versus host-time policy and clock-offset diagnostics;
- composable-node support and intra-process communication;
- protocol parser and sweep-assembly tests using recorded packet fixtures.

Projection, gain/normalization, thresholding, CFAR, Cartesian images, laser
scans, and point clouds should remain downstream operations. That lets every
consumer begin from the same unchanged `uint16` FFT values.

## Recommended implementation path

1. Pin a specific canonical IASDK commit and document its MIT attribution.
2. Create `navtech_radar_msgs` and adapt this bag's legacy
   `HighPrecisionFFTData` into the proposed spoke message. (Implemented.)
3. Validate sweep assembly, timestamps, and drop detection against this bag.
   (Implemented for playback; hardware packet fixtures remain.)
4. Add the live Colossus transport node when CTS350-X hardware is available.
5. Derive Cartesian images, CFAR clouds and scans from the raw stream. (Implemented for playback.)

The bag adapter provides a repeatable test source before live hardware is
connected and prevents the production API from inheriting the shortcomings of
the legacy ROS 1 message.
