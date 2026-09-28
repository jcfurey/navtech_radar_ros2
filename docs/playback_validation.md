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
