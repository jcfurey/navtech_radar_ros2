# Playback validation (2026-09-28)

Validated on ROS 2 Lyrical, Python 3.14.4, NumPy 2.3.5. The changes affect the
legacy adapter and sweep assembler; a live radar transport is still unimplemented.

## Automated checks

The driver and processing packages register pytest suites through
`ament_cmake_pytest` and `ament_cmake_ros`. Tests instantiate the actual nodes
and generated message classes. Publication capture serializes and deserializes
outputs to check message representation. Separate DDS tests exercise namespace
resolution, configuration durability, and the configuration/spoke/sweep path.
No radar hardware or recorded bag is required for these tests.

## Recorded-data check

The first 16,000 radar spokes in
`tamu_full_stack_route_1_run_2_2024-04-23-10-27-20_ros2_mcap` were read with
`rosbag2_py.SequentialReader`, adapted, and passed into the assembler callbacks.
Every emitted image row was compared against its original spoke using sensor
timestamp and sweep counter. Encoder counts, host stamps, image dimensions,
little-endian uint16 samples and ROS serialization were also checked.

Results:

- One distinct configuration: 400 azimuths, encoder size 5600, 6000 bins,
  approximately 0.0438 metres/bin, 4 revolutions/second.
- 40 published sweeps: one initial partial sweep of 234 rows and 39 sweeps of
  400 rows. The final 166 spokes remained buffered pending the next wrap.
- No rejected spokes or discarded sweeps in this segment.
- Every published FFT sample and checked metadata field matched its source.

This validates recorded-message conversion and assembly. It does not measure
DDS delivery of the full bag, the complete recording, or live hardware behavior.

## Assembly performance

Compared against `abf90aa`, using the same 16,000 decoded `FftSpoke` messages
and their recorded configuration. Publishers were replaced with no-op sinks to
isolate callback and image construction cost. Point-cloud generation was off.
Each timing is the median of three runs with a fresh node. A separate run with
`tracemalloc` measured peak allocations during callbacks; already decoded input
messages were outside the measurement.

| Measurement | Before | After |
| --- | ---: | ---: |
| Callback time for 16,000 spokes | 109.35 ms | 79.03 ms |
| Peak traced callback allocations | 19,629,260 bytes | 10,155,884 bytes |

On this machine, this is about 28% less callback time and 48% lower peak traced
allocation. It is not an end-to-end ROS throughput benchmark. The improvements
come from retaining read-only NumPy views of ROS uint16 buffers, directly filling
the ROS image byte array, and avoiding projection for discarded sweeps. The last
optimization is not exercised by this point-cloud-disabled benchmark.

## SDK review

The pinned IASDK commit is still `62759ba2f60bcef6e3787d889aba6b0773dde647`.
Neither active Python node imports or links it, so playback fixes do not require
changing the SDK gitlink or applying local SDK patches.

The C++ `FFT_data::timestamp_sec()` / `timestamp_nsec()` accessors and the
recorded legacy messages use seconds and nanoseconds. The adapter should not
apply the standard NTP epoch or binary-fraction conversion merely because the
ROS field names contain `ntp`. Original wire bytes cannot be reconstructed from
these legacy messages.

Before implementing live transport, enforce packet boundaries before using the
SDK's C++ overlay API. In `Colossus_TCP_network_message.cpp`, `Message::is_valid()`
only checks `data.empty()` before reading the signature/header, and
`payload_size()` has the same empty-only guard. In
`Colossus_TCP_network_message.h`, `view_as<T>()` directly reinterprets the
buffer. These APIs should not be called on unvalidated truncated input.
`Header_and_payload::to_vector()` also uses the fixed typed header size, while
FFT packets expose `fft_data_offset()`. Future live-transport fixtures should
cover short headers, advertised versus received payload length, FFT offsets,
odd-length 16-bit payloads, and oversized frames before any overlay/allocation.

These are integration constraints found by source inspection, not a claim that
the SDK's normal TCP receiver passes truncated buffers. The SDK's bundled ROS
examples remain excluded from colcon and are not the production ROS API.

## Heading, adaptive detections and LaserScan (2026-10-01)

Built `navtech_radar_processing` and `tamu_bag_viz` on ROS 2 Lyrical. The
registered suites passed: 83 processing tests and 17 driver tests. Added cases
cover additive CFAR against explicit scalar training windows, unsigned sample
overflow, clutter transitions, peak separation, scan nearest/strongest policy,
the independent cloud point budget, duplicates and missing bearings, partial
scan angle/stamp alignment, acquisition timing, azimuth offsets, and timestamped
TF heading queries including an inverted radar mount.

The first 4,000 recorded spokes were passed through the real adapter and
assembler callbacks with CFAR, cloud and scan outputs enabled. A
`rosbag2_py.SequentialReader` also supplied recorded TF to the visualizer's
buffer. The existing placeholder mount was added as a static transform. Each
emitted FFT row was compared with the original bag sample, indexed by sensor
timestamp and sweep counter. ROS serialization, scan ranges/angles/frame,
coverage and point counts were checked.

Results:

- 10 sweeps: one initial 234-row partial sweep, then nine 400-row sweeps;
  166 spokes remained buffered. No rejected spokes or discarded sweeps.
- All emitted uint16 FFT samples and checked encoder values matched the source.
- The partial scan had 166 missing bearings; each complete scan had none.
- Complete sweeps retained 808–842 cloud points and 324–337 finite scan returns
  with the default CFAR settings. The last sweep retained 808 cloud points,
  versus 3,316 with fixed-threshold extraction and the same 48-point budget.
  This measures selection density, not accuracy against labeled targets.
- `scan_time=0.25` s. Maximum timestamp deviations from the nominal 625 µs
  angular cell period ranged from about 71 to 388 µs per sweep. Every sampled
  sweep therefore used `time_increment=0` rather than a uniform motion model.
- Map yaw was available on eight of the ten sweeps and the vehicle-forward
  transform on nine. Early sweeps precede enough recorded TF history; missing
  heading remains explicitly unavailable.

The previews [plan view](radar_heading_plan.png) and
[range–azimuth image](radar_heading_polar.png) show the final complete sweep with
timestamped TF annotations. Yaw is relative to `lester/map`; the radar mount
remains a visualization placeholder, so these are not calibrated geographic
heading or radar-to-lidar alignment results.

A headless installed launch in isolated local domain 92, with
`radar_namespace=front`, `radar_scan=true` and `radar_point_cloud=true`, delivered
five messages on each of `/front/navtech_radar/scan`,
`/front/navtech_radar/point_cloud` and `/front/navtech_radar/fft/cartesian` to
real DDS subscribers. Message dimensions, range count, period and radar frame
were checked. All launch processes exited cleanly after a single parent SIGINT.
This exercises the installed pipeline and namespace handling, but does not
validate the entire bag, detection accuracy, or the RViz GUI.

## Planar mounting correction (2026-10-01)

The cloud appeared horizontally mirrored relative to the correct radar-frame
images. The provisional LWLRF-derived roll of 3.14 rad reverses the mounting
frame's +y direction: a return at radar `(0, 10, 0)` becomes approximately
`(-0.225, -9.997, 0.016)` after that rotation and the -0.0225 rad yaw, before
translation. With roll 0, it becomes `(0.225, 9.997, 0)`, preserving left/right.

The workspace launch and robot-description defaults now use roll 0. The
[workspace patch](tamu_planar_mount.patch) preserves both edits in this driver
repository. The historical `LWLRF` source is unchanged. This corrects the
display convention; translation and yaw still require physical calibration.
That change addressed only the mount. The encoder sign was subsequently checked
against vehicle motion as described below.

Validation expands the installed robot Xacro, checks agreement with the launch
default, and verifies that +y retains positive mounting-frame y. A second
headless DDS playback check also verifies the live static TF has zero roll and
pitch while delivering the namespaced radar scan, cloud and image products.

## Native playback comparison

On November 17, 2026, both installed implementations processed the first partial
revolution and the next 32 complete revolutions of the April 2024 route 1/run 2
recording on ROS 2 Lyrical (Linux x86-64, CMake Release). The partial revolution
served as warmup. Both runs used 400 spokes × 6,000 uint16 samples per full
revolution, identical CFAR parameters, a 48-point cloud budget per spoke and
nearest-return scans.

| Measured time for 32 full revolutions | Python | C++ |
| --- | ---: | ---: |
| Adapter CPU | 4.27 s | 1.60 s |
| Assembler CPU | 6.93 s | 2.56 s |
| Combined node CPU | 11.20 s | 4.16 s |
| Elapsed including feeder, DDS, and output checks | 6.65 s | 3.41 s |

The [comparison script](../tools/compare_playback.py) preloads the bag's spokes,
publishes the recorded configuration and verifies all four outputs with a probe
before sending bounded bursts of recorded legacy FFT messages.
It waits for the sweep, mono16 image, point cloud, and scan after each wrap.
CPU measurements use the two child processes' user and system time with
subscribers active. The feeder, file I/O, image renderer and RViz are outside
the combined CPU measurement. This is a single-machine comparison for this
workload. Whole-bag playback also carries the other sensors and repeated
configuration messages.

All 33 recorded raw images matched the source samples byte-for-byte, including
rows with equal encoder counts in their arrival order. Both implementations
agreed on sweep metadata, relative sequence numbers, scan fields, and the exact
range and intensity array bytes. Cloud sizes, intensity and
z matched exactly, with xy checked at 2e-6 relative / 2e-5 m absolute tolerance.
Both adapters used the same frozen simulated clock for comparable receipt stamps.

The registered tests include a synthetic installed-executable comparison covering
an initial partial sweep, duplicate and missing bearings, full regular timing,
CFAR and fixed thresholds, nearest/strongest scan selection, azimuth offset and
cloud budget. Run the [standalone build and tests](../README.md#standalone-playback-on-ros-2-lyrical)
for the configuration used here.

A fresh clone built all four packages against the Lyrical underlay with the
optional IASDK submodule uninitialized. `rosdep check` found all dependencies
and `colcon test-result` reported zero failures. Running the installed
`playback.launch.py` on the same MCAP delivered five 400-ray scans, five
nonempty XYZI clouds and five 800 × 800 Cartesian images to a headless DDS
subscriber. All three products used the `navtech` frame.

## Native image comparison

The [image comparison script](../tools/compare_images.py) feeds the installed C++
and Python visualizers identical sweeps from the route 1/run 2 recording. On
ROS 2 Lyrical, both produced identical headers, dimensions, encodings, strides
and pixel bytes for the annotated polar and Cartesian views on 32 full revolutions.
With the default display settings, the C++ visualizer used 1.49 s of process CPU
and the Python visualizer used 2.51 s. Each process had subscribers for both
views. Measurement started after an initial partial sweep warmed the projection
cache. Bag decoding, sweep assembly, the comparison process and RViz were
outside the measurement. The replay used empty heading frame parameters.

The installed-executable tests also compare single-view subscribers, padded
big-endian samples, azimuth offsets, duplicate and missing rows, cropped range
pooling, contrast history across rewinds and configuration changes, and gamma.
A timestamped TF fixture checks the arrow and heading text against an interpolated
45-degree vehicle yaw, then seeks before the transform history and checks the
unavailable-heading display.

## Bearing and range validation

A near-stationary turn in the April 2024 TAMU route 1/run 2 recording gives an
independent check on left and right. Near 869 s, the vehicle translated 0.014 m
and yawed −20.314° between two sweep midpoints one second apart, using
interpolated `/lester/odom` poses. Stationary surroundings should therefore
move toward positive body bearings. Cross-correlation of the two installed
renderers' plan views measured a +20.700° scene shift, within one 0.9° sample.
This tests handedness on the zero-roll TAMU installation. The constant mounting
yaw and translation still require an external calibration.

With the standalone workspace sourced, run the check on the original MCAP
directory. It compares both executables' Cartesian pixels, samples the 12–45 m
annulus in counterclockwise ROS bearings, and correlates the azimuth profiles
after standardizing each range column. [Regression script](../tools/check_tamu_turn.py)

```bash
python3 tools/check_tamu_turn.py /path/to/tamu_full_stack_route_1_run_2_2024-04-23-10-27-20_ros2_mcap
```

The legacy recording supplies nominal spacing only. A separate canonical DDS
fixture uses 0.5 m nominal spacing, gain 2 and offset +1 m. Bins 2 and 4 appear
at 3 m and 5 m on both installed assemblers, and the 7 m return lies outside
the selected range. Other fixtures cover negative offsets, metric CFAR peak
separation, the calibrated image projection, empty scan rays and zero-stamp TF.
The recorded previews above use the corrected bearing with the zero-roll mount.
