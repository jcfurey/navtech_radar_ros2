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
```

The TAMU workspace's `view_bag.sh` launches this pipeline automatically.

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
only the visualization point cloud, not the canonical FFT data.

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
