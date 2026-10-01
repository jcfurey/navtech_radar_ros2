# Navtech Radar ROS 2

Lossless ROS 2 support for Navtech rotating radar, initially targeting the
CTS350-X and the 16-bit high-precision FFT stream used by The Great Outdoors
Dataset.

## Status

This repository pins the Navtech IASDK and implements the lossless ROS API,
legacy bag playback transport, and sweep assembly. A live IASDK transport node
has not been implemented yet.

The intended boundary is:

- preserve every high-precision `uint16` FFT spoke and its sensor metadata;
- keep the Navtech Colossus transport separate from ROS messages;
- build sweep assembly, Cartesian projection, CFAR, and point clouds as
  downstream components;
- leave radar-to-robot TF ownership to the robot description or calibration
  package.

See [docs/driver_design.md](docs/driver_design.md) for the research findings,
proposed messages, QoS, and implementation sequence.
The [hardware reference](docs/cts350x.md) includes Navtech's official CTS350-X
datasheet and compares its standard settings with the recorded configuration.

## Bag playback

`navtech_radar_driver/legacy_bag_driver` adapts the dataset's ROS 1-derived
topics without changing the source bag:

```text
/Navtech/Configuration_Topic -> /navtech_radar/configuration
/Navtech/FFTData             -> /navtech_radar/fft/spoke
```

`navtech_radar_processing/fft_sweep_assembler` then publishes:

```text
/navtech_radar/fft/sweep     navtech_radar_msgs/FftSweep
/navtech_radar/fft/image     sensor_msgs/Image (mono16)
/navtech_radar/point_cloud   sensor_msgs/PointCloud2 (optional, lossy)
/navtech_radar/scan          sensor_msgs/LaserScan (optional, lossy)
```

The TAMU workspace's `view_bag.sh` launches this pipeline automatically.

## Enhanced images

`navtech_radar_processing/fft_sweep_visualizer` optionally consumes the sweep
and its configuration and publishes two `bgr8` display images:

- `navtech_radar/fft/display`: cropped, labeled range–azimuth image.
- `navtech_radar/fft/cartesian`: top-down radar image with range rings, bearing
  ticks, +x up and +y left. Both images show coverage, missing/duplicate bearings,
  frame name and scan rate. TF supplies a cyan vehicle-forward marker and vehicle
  yaw in the requested reference frame at the sweep's acquisition timestamp.
  If that transform is unavailable, yaw is labeled unavailable.

Set `robot_frame` and `heading_frame` to enable the TF overlays; the TAMU launch
sets `lester/base` and `lester/map`. Yaw is counterclockwise around reference +z
from reference +x, not a geographic compass heading. TF is used for annotations;
image pixels stay in the radar frame. The full mounting rotation is respected
when projecting the vehicle's forward axis into that frame.

The default display range is 2–150 m. Range columns are reduced to at most
1000 using maximum pooling, preserving narrow peaks. Encoder values place rows
on the configured azimuth grid; duplicate rows combine by maximum for display,
and missing rows remain black. Smoothed 50th/99.7th percentile contrast and an
Inferno color scale make returns easier to see than full-range linear grayscale.
The color scale shows raw FFT levels, not calibrated power or height. Contrast
history resets when configuration changes or timestamps move backward.

These are lossy visualization products. `FftSweep`, `FftSpoke`, and the original
`mono16` image are unmodified. The radar cloud stays planar because the source
contains no elevation measurement. The visualizer requires a matching nonzero
configuration version and supports up to 2048 azimuth rows and 16384 range bins.
Image publishers and the sweep subscription use reliable, volatile QoS, depth 1.
Rendering is skipped when neither output has subscribers.

```bash
ros2 run navtech_radar_processing fft_sweep_visualizer --ros-args \
  -p use_sim_time:=true -p max_range:=100.0 -p lower_percentile:=60.0
```

Other startup parameters include `min_range`, `image_width`, `cartesian_size`
(800 by default), `upper_percentile`, `contrast_alpha` (0.2), `gamma` (1.0),
and `annotate` (true). Gamma below 1 brightens weaker returns. The playback
workspace starts this node and configures RViz automatically. Its
`radar_images:=false` launch option returns to the raw image display.

`azimuth_offset_degrees` rotates the derived Cartesian projection and polar
bearing labels. The TAMU launch shares this value with cloud and scan conversion.
The raw encoder grid and all canonical messages remain unchanged.

![Recorded FFT sweep before and after display processing](docs/fft_display_comparison.png)

Updated recorded-data previews: [plan view with heading](docs/radar_heading_plan.png)
and [range–azimuth view](docs/radar_heading_polar.png).

## Planar detections and scans

The assembler can publish both multiple detections per spoke as `PointCloud2`
and one detection per bearing as `LaserScan`. Enable `publish_point_cloud` and/or
`publish_scan`. Both retain the input radar frame and raw FFT intensity units;
cloud z is zero. Mounting is supplied by TF.

`peak_selection=threshold` preserves the original local-maxima threshold mode.
The TAMU launch defaults to `cfar`: cell-averaging background estimation with
16 training cells and 4 guard cells **per side**, an additive threshold offset
of 3000 raw FFT units, and the existing 22000 minimum intensity. Complete
training windows are required. Maxima closer than `min_peak_separation` (0.5 m)
are reduced to the strongest, with ties preferring nearer bins. These are
empirical settings for this 16-bit recording; additive raw-level filtering
does not imply a calibrated false-alarm probability. Clutter boundaries can
still produce detections.

Clouds retain the strongest `max_points_per_spoke` peaks (0 means all), in range
order. `scan_selection=nearest` chooses the nearest filtered peak before that
cloud budget is applied; `strongest` instead chooses the largest FFT level.
Thus a scan is useful for a compact planar view, while the cloud retains
multiple reflectors along a bearing.

The scan has the configured number of bearings, positive angle increment, and
no duplicated 360° endpoint. Its first angle is the first acquired spoke's
bearing, including the derived azimuth offset, and its header is that spoke's
sensor timestamp. Initial partial sweeps therefore may start at a nonzero angle;
`angle_max` may exceed 2π. Missing bearings are `NaN`; observed bearings with no
selected return are `+inf`, with intensity 0. Off-grid rays farther than a
quarter angular cell are omitted instead of interpolated into gaps. Duplicate
bearings use the selected nearest/strongest policy.

`scan_time` comes from the configured rotation period. `time_increment` is
nonzero only when per-spoke timestamps agree with that uniform acquisition
schedule within 10% of a cell period (at least 1 µs), without duplicates or
off-grid rays. Otherwise it is 0, representing a snapshot. This conversion is
lossy radar detection data: a missing detection is not evidence of free space,
and LaserScan consumers can make assumptions specific to laser sensing.

```bash
# TAMU workspace: turn on both RViz displays and their publication.
./view_bag.sh radar_scan:=true radar_point_cloud:=true
# Select the strongest radar reflector per scan bearing.
./view_bag.sh radar_scan:=true radar_scan_selection:=strongest
# Reproduce the original fixed-threshold extraction.
./view_bag.sh radar_point_cloud:=true radar_peak_selection:=threshold
```

Other extraction parameters are `cfar_training_cells`, `cfar_guard_cells`,
`cfar_threshold_offset` and `min_peak_separation`; the TAMU launch prefixes
these with `radar_`. Set a suitable intensity floor and CFAR offset for other
sample depths or recordings. Scan/cloud publishers use sensor-data QoS and
publish empty/all-no-detection results to clear previous sweeps.

## Runtime behavior

Canonical topic defaults are relative (`navtech_radar/configuration`,
`navtech_radar/fft/spoke`, etc.). At the root namespace they retain the names
shown above. In namespace `/robot`, they become `/robot/navtech_radar/...`.
Diagnostics use the relative `diagnostics` topic. This follows ROS 2's
[relative topic name semantics](https://design.ros2.org/articles/topic_and_service_names.html#namespaces).
The two legacy input defaults remain absolute to match existing bags; override
the input topic parameters or use ROS remapping for other sources.

Configuration uses reliable, transient-local QoS, depth 1. Spokes use
best-effort, volatile QoS with a configurable bounded queue. Sweep and image
outputs use reliable, volatile QoS, depth 2; optional point clouds use the
sensor-data profile. "Lossless" describes the sample representation, not a
guarantee of packet delivery over best-effort DDS.

The adapter rejects malformed payload sizes, invalid timestamps and invalid
configuration geometry. A spoke before the first configuration has version 0.
The legacy seconds/nanoseconds are preserved; the bag cannot recover original
wire timestamp bytes. Diagnostic timers use steady time so they continue to
report stale input while `/clock` is absent or paused.

The assembler preserves the incoming frame. `output_frame` defaults to empty;
if set, it requires that same input frame. It does not transform coordinates.
Use TF for mounting transforms. The optional `azimuth_offset_degrees` adjusts
derived scan/cloud bearings, not the canonical FFT data. Give the visualizer
the same value to keep its Cartesian view aligned.

The TAMU workspace's provisional mount uses `radar_mount_roll=0.0` so the
radar's +y axis remains leftward in the robot frame. A roll of approximately π
around +x reverses +y and +z, horizontally mirroring a planar cloud/scan even
when the radar-frame images look correct. The earlier LWLRF-derived 3.14 rad
roll was removed following that observed mismatch. Translation and yaw remain
provisional; a physical calibration should replace the placeholder transform.
The workspace correction is preserved in [docs/tamu_planar_mount.patch](docs/tamu_planar_mount.patch)
because the surrounding TAMU playback workspace is not part of this repository.
Apply that patch from a workspace root still using the old roll, then rebuild
`lester_description` and `tamu_bag_viz` and restart playback. It updates both
the static TF launch default and the robot model's matching joint default.

Sweeps never mix configuration versions, range-bin counts, sample depths, or
frames. Configuration changes finish the preceding sweep with its original
geometry. Nonzero versions are accepted only after their configuration arrives;
unmatched spokes are rejected with a warning rather than assigned guessed
geometry. Version 0 uses `range_resolution` and `expected_spokes` as fallback
values until a configuration arrives. Invalid configurations leave the last
valid configuration active.

Rows are stably sorted by encoder count and duplicates remain separate rows.
`publish_partial_sweeps` defaults to true, subject to `minimum_spokes` (2).
When false, a sweep must contain exactly `expected_spokes` unique encoder counts
and no duplicates. This count check cannot prove angular coverage of a malformed
stream. A backward sensor timestamp discards the pending sweep, allowing bag
loops or seeks to start a new sweep. The final pending sweep remains buffered
until a wrap or configuration/geometry change; shutdown does not publish it.

`max_buffered_spokes` (2048) and `max_range_bins` (16384) bound retained raw data
to at most 64 MiB with the defaults, excluding DDS queues and output buffers.
Oversized spokes/configurations are rejected. Reaching the spoke limit without
a wrap discards the pending sweep and starts again. Increase the limits at
startup for larger supported scans. Empty point clouds are published when an
accepted sweep has no selected peaks, allowing viewers to clear stale points.

All application parameters are validated at startup and read-only afterward;
`use_sim_time` retains the standard ROS behavior. Restart the node to change
processing settings. ROS topic remapping and parameter overrides still apply
at startup.

## Build and tests

From a workspace containing this repo and the legacy `nav_ross` interfaces:

```bash
colcon build --packages-up-to navtech_radar_driver navtech_radar_processing \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select navtech_radar_driver navtech_radar_processing
colcon test-result --verbose
```

The tests use real generated ROS messages, serialization, and isolated ROS
domains. They cover payload preservation, configuration transitions, malformed
inputs, bounded buffering, rewind, partial-sweep policy, point-cloud bytes,
namespace handling, late subscribers, and paused simulated time. See
[docs/playback_validation.md](docs/playback_validation.md) for recorded-data
validation, performance measurements, and SDK integration findings.

## SDK submodule

The canonical Navtech IASDK is pinned at `vendor/iasdk-public` as a Git
submodule, currently at commit `62759ba2f60bcef6e3787d889aba6b0773dde647`:

```bash
git submodule update --init --recursive
```

The SDK is independently MIT licensed by Navtech Radar Ltd. See
`vendor/iasdk-public/LICENSE.txt` after initializing the submodule and
[THIRD_PARTY.md](THIRD_PARTY.md).

The `vendor/COLCON_IGNORE` marker prevents colcon from discovering the SDK's
bundled ROS examples as workspace packages. Our driver will consume only the
needed C++17 Colossus protocol and transport targets.

### Upstream checkout note

The pinned upstream commit has a line-ending defect in
`cpp/cpp_17/.gitattributes`: three prebuilt Windows `.lib` files and two CMake
files appear modified immediately after a clean Linux checkout. The submodule
is treated as read-only, and `.gitmodules` suppresses that known false-positive
in the parent repository. Inspect its actual state explicitly when needed:

```bash
git -C vendor/iasdk-public status --short
```

Any SDK change should be made in a reachable fork/commit and then update the
gitlink; local uncommitted SDK patches must not become driver dependencies.

## Repository layout

```text
navtech_radar_ros2/
├── docs/
├── navtech_radar_driver/       # transport nodes and legacy bag adapter
├── navtech_radar_msgs/         # configuration, spoke, and sweep interfaces
├── navtech_radar_processing/   # optional sweep/projection/CFAR components
└── vendor/
    └── iasdk-public/           # pinned upstream submodule
```
