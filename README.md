# Navtech Radar ROS 2

Lossless ROS 2 support for Navtech rotating radar, initially targeting the
CTS350-X and the 16-bit high-precision FFT stream used by The Great Outdoors
Dataset.

## Status

This repository currently pins the Navtech IASDK and records the driver/API
design. The ROS interfaces and live transport node have not been implemented
yet.

The intended boundary is:

- preserve every high-precision `uint16` FFT spoke and its sensor metadata;
- keep the Navtech Colossus transport separate from ROS messages;
- build sweep assembly, Cartesian projection, CFAR, and point clouds as
  downstream components;
- leave radar-to-robot TF ownership to the robot description or calibration
  package.

See [docs/driver_design.md](docs/driver_design.md) for the research findings,
proposed messages, QoS, and implementation sequence.

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

## Planned repository layout

```text
navtech_radar_ros2/
├── docs/
├── navtech_radar_driver/       # lifecycle/composable transport node
├── navtech_radar_msgs/         # configuration, spoke, and sweep interfaces
├── navtech_radar_processing/   # optional sweep/projection/CFAR components
└── vendor/
    └── iasdk-public/           # pinned upstream submodule
```

The package directories will be introduced as their public contracts are
implemented and tested against the TAMU bag.
