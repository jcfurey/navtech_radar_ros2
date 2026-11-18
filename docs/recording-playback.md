# Playing public radar recordings

In a sourced workspace, pass the radar file, sequence directory, or ROS 2 bag directory to the same launch. The following commands assume `data` points at the collected `navtech-finds` directory.

```bash
data="/run/media/jcfurey/2TB/Test Datasets/navtech-finds"

ros2 launch navtech_radar_processing playback.launch.py \
  recording:="$data/radiate/tiny_foggy" rviz:=true

ros2 launch navtech_radar_processing playback.launch.py \
  recording:="$data/muses/radar_trainvaltest.zip" max_gap:=0.25 rviz:=true
```

Run one launch at a time. `rate:=1` follows recorded time. For file inputs, `max_gap` caps the delay between successive measurements in source seconds before applying the rate multiplier. This is especially helpful for the independent MUSES scenes. The source timestamps still appear in message headers. `limit:=20` plays the first 20 rotations, standalone images, or ROS 1 sensor messages. ROS 2 bags use the native rosbag2 player and support `rate`. Press Ctrl-C to close the processing nodes and RViz after the last frame.

## Selecting an input

Use one entry per launch. Paths below are relative to `data`. For entries containing a wildcard, select a matching file or sequence directory. An explicit `recording_format` is useful for extracted directories that have been renamed.

| Recording | Input path | Format and result |
| --- | --- | --- |
| TAMU CTS350-X | `tamu-cts350x-ros2` | ROS 2 MCAP. The legacy adapter reads the 16-bit FFT messages and recorded TF. `bag:=` also works. |
| Navtech road, sea fog, low-light port and reclaimer | One `.colraw` in `navtech-road`, `navtech-mas10-fog` or `navtech-reclaimer` | `colraw`. Reads the recorded configuration, packet counters, encoders and FFTs, including compressed records and partial rotations. The reclaimer scanned a vertical plane. Its coordinates are relative to the radar mount. |
| Navtech Thames RAS6 | The original `.cpr` in `navtech-ras6-thames` | `cpr`. Requires the same-stem `.colraw` alongside it. The collected sample includes this companion. |
| Navtech road ROS 1 products | `navtech-road/ROS1.zip` or the extracted `.bag` | `ros1`. Publishes the original polar image, Cartesian image, filtered point cloud and scan, preserving their message headers and fields. Uses bag time for scheduling. |
| Navtech road ROS 2 products | One bag directory in `navtech-road/ros2` | Each bag holds one published product. The image and scan are remapped directly. The known thresholded-cloud export is converted to metric ROS XYZI. Each directory contains its own `metadata.yaml` and SQLite file. |
| Oxford Radar RobotCar | `oxford-radar-robotcar/oxford_radar_robotcar_dataset_sample_tiny.zip` | `oxford`. Uses the per-row timestamp, measured encoder and validity byte. |
| Oxford Offroad (Bellmouth) | `oxford-offroad/Navtech_CTS350-X_Radar.tar.gz` or its extracted radar folder | `oord`. Per-row timestamps and packet counters. Uses Oxford's row-angle convention, including for frames with 399 rows. |
| RADIATE | `radiate/tiny_foggy` | `radiate`. Uses the polar PNGs and timestamps to generate the driver products. |
| RADIATE original Cartesian images | `radiate/tiny_foggy/Navtech_Cartesian` | `radiate_cartesian`. Replays the authors' 1,152 × 1,152 mono8 images and timestamps on `/navtech_radar/fft/cartesian`. |
| Boreas and Boreas Road Trip | `boreas/<sequence>/radar` or `boreas-road-trip/<sequence>/radar` | `boreas` and `boreas_rt`. Both read the per-row encoder and time. Road Trip retains both chirps and exposes the original direction bytes on the recording input topic. |
| MulRan ParkingLot | `mulran/ParkingLot/polar.zip` or `mulran/ParkingLot/ray.zip` | `mulran` uses the uniform polar grid and frame timestamps. `mulran_ray` uses the measured encoder, packet counter and nanosecond time on every CSV row. The two releases contain different numbers of frames. |
| MUSES | `muses/radar_trainvaltest.zip` | `muses`. Orders scenes by the release metadata. Reads azimuths from columns, with separate seconds and nanoseconds and the measured encoder per column. |

The Oxford Offroad tar is shuffled. Playback stages its radar PNGs in timestamp order and needs about 6 GB of temporary space. The ROS 1 ZIP uses Deflate64 and stages an 8.35 GB bag. Set `TMPDIR` to a filesystem with room, or extract once and play the radar folder or bag directly for repeated runs. The other listed ZIPs can be read in place.

## Measurements and derived products

For FFT recordings the Python reader publishes `/navtech_radar/recording/rotation` with the source bit depth, acquisition order, configuration, bearings and available row metadata. The C++ assembler generates the normal encoder-sorted raw sweep and mono16 image, Cartesian and polar views via the visualizer, and optional `PointCloud2` and `LaserScan`. An 8-bit value of 200 remains 200 in the mono16 image. The sweep's `sample_bit_depth` identifies the source precision. Rows marked invalid or interpolated in the source remain available on the recording input topic. The assembler omits those rows from its measurement products. The Road Trip chirp byte has a different meaning and both values produce measurements.

The 8-bit detection threshold and CFAR offset default to 24 and 12 intensity units. Use `recorded_threshold_8bit` and `recorded_cfar_offset_8bit` to adjust the exports. The 16-bit pipeline retains its existing settings. The file reader uses the C++ assembler for all three launch implementations. `implementation:=python` selects the Python visualizer for FFT file playback.

RADIATE and MulRan polar PNGs only carry frame times. Their sweeps use the frame timestamp for each row, and their scans report zero time increment. The reclaimer files have an unset sensor clock, so their headers use elapsed recorder time. For Thames, select the CPR to use the original June 2024 recorder time. Navigation logs, ground-truth poses, and extrinsic calibration files in the collection are separate inputs. Heading and deskew require a timestamped TF supplied by a bag or by the user.

Oxford projections use 0.0432 m bins with the first centre at 0.0216 m. Boreas uses ASRL's resolution upgrade date and default −0.31 m range correction. MUSES places the first range bin at one range increment. Angles and ranges refer to the radar's own frame. [Oxford RobotCar SDK](https://github.com/ori-mrg/robotcar-dataset-sdk/blob/master/python/radar.py) · [Oxford Offroad SDK](https://github.com/mttgdd/oord-dataset/blob/main/src/radar.py) · [Boreas radar loader](https://github.com/utiasASRL/pyboreas/blob/master/pyboreas/utils/radar.py) · [ASRL calibration](https://github.com/utiasASRL/pyboreas/blob/master/pyboreas/data/calib.py) · [MUSES radar processing](https://github.com/timbroed/MUSES/blob/master/processing/radar_processing.py)

### Navtech road cloud coordinates

The ROS 1 `/Navtech/FilteredPointcloud` contains metric points in `navtech` with a 32-byte stride and scalar intensity at offset 16. Playback preserves its fields, padding, intensity values, frame and timestamp.

The manufacturer's original ROS 2 `RadarDataFromVehicle_ThresholdedPointCloud` SQLite bag stores clockwise Cartesian coordinates in **range bins**. Each 16-byte point holds four scalar floats, although each field's recorded count equals the cloud width. On `/navtech_radar/point_cloud`, the launch publishes scalar `x, y, z, intensity` fields with count 1, scales x by 0.0438 m/bin, and scales y by −0.0438 m/bin to put positive y on the left. It uses the requested radar `frame_id` (default `navtech`). The coordinates retain the exporter's nominal 0.9° bearing grid. The timestamp is the recorded wrap-ray timestamp at the end of the accumulated rotation. Point order, zero-range returns and raw intensity are preserved. The native FFT path supports measured encoder bearings, range gating and per-spoke timing. [Validation and reproduction](playback_validation.md#filtered-point-cloud-validation) · [Manufacturer's road example](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2061009475)

The launch identifies this export by the SHA-256 of the SQLite member in Navtech's public ZIP, so the correction also works when the containing directory is renamed. Repacked or trimmed bags have different checksums and use the ordinary topic remap. For those copies, use the original export or the accompanying COLRAW for validated metric playback. Direct `ros2 bag play` publishes the stored vendor message on `/radar_data/point_cloud`.

### Preparing the Thames companion

Navtech's Windows CPR-to-Colossus converter decodes the FFT amplitudes. For the **Tower Bridge sample**, use the input filename with `-f`, 5,600 encoder ticks with `-e 5600`, 400 azimuth samples with `-a 400` and disable output compression with `-c 1`. Rename its generated file to the original CPR's basename with the extension `.colraw`. The converter writes new timestamps into its output. Opening the CPR in the driver pairs each converted FFT with the original video record, checks the bearings within one encoder tick, and uses that record's capture time. [Navtech conversion guide](https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/2359459855) · [Converter download](https://navtechradar.atlassian.net/wiki/rest/api/content/2810249244/child/attachment/att3866034189/download) · [Tower Bridge recording](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2794651653)

## Playback checks on the collected files

- The complete TAMU MCAP yielded 2,962,148 FFT messages and 2,724,312 configuration messages, matching the bag metadata. Each spoke carried 6,000 16-bit samples, with a matching size field and an encoder in range.
- The eleven original COLRAW captures decoded through EOF, yielding 10,308 rotations including partials and 4,077,568 FFT spokes. All 243,212 converted Thames spokes matched their paired CPR bearing.
- Whole-file reads covered 7,097 Oxford Offroad frames, 2,500 MUSES frames, 477 MulRan polar frames, 470 MulRan ray frames, 11 Oxford RobotCar frames, 18 RADIATE polar frames and 32 frames in each Boreas sample.
- At 8× playback, ROS subscribers received all 6,166 road ROS 1 products and all 4,440 products from the three road ROS 2 bags. All products retained their recorded timestamps and payload lengths. The normalized ROS 2 cloud uses the selected radar frame and metric coordinates.
- The installed ROS 2 launch delivered sample messages for every recording, including TAMU and each of the three vendor ROS 2 bags. At recorded speed, all six FFT-derived topics delivered every frame of RADIATE (18/18) and Boreas Road Trip (32/32). The raw sweep samples and acquisition timestamps matched the source PNGs through the final frame, and all 18 published RADIATE Cartesian images matched their original pixels and timestamps.

The [source inventory](radar-products-and-recordings.md) gives the model identifications, downloads, and dataset credits.
