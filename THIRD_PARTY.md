# Sources and acknowledgments

## Navtech Radar

Navtech Radar Ltd manufactures the CTS350-X and publishes the Colossus protocol
and IASDK examples. The live sensor protocol, configuration fields, and 16-bit
high-precision FFT mode described in the design notes come from these resources.

- [Radar data and software connection](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2060812439)
- [Manufacturer's SDK documentation](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/3506151435/SDK) and [ROS 2 examples](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2323349527)
- [CTS350-X technical datasheet, ITS-PM-0001 V2.2](https://navtechradar.com/wp-content/uploads/2024/12/CTS350-DATA-SHEET-V2.2.pdf). The [local reference](docs/cts350x.md) identifies the archived PDF, its checksum, and the configuration recorded by TAMU. The PDF retains the manufacturer's notices.
- [Published works including Navtech Radar](https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2408480879/Published+works+including+Navtech+Radar) lists this project's high-precision dataset playback support.

The optional submodule at `vendor/iasdk-public` pins
[Navtech IASDK revision 62759ba](https://bitbucket.org/navtechradar/iasdk-public/src/62759ba2f60bcef6e3787d889aba6b0773dde647/).
It is copyright © 2024 Navtech Radar Ltd and distributed under the MIT license.
Its copyright and permission notice is in `vendor/iasdk-public/LICENSE.txt`.
The playback executables build and run independently of the submodule.

## Dataset and legacy ROS messages

The Texas A&M Unmanned Systems Lab and its collaborators collected and published
[The Great Outdoors Dataset](https://github.com/unmannedlab/the-great-outdoors-dataset).
The recording, radar configuration, legacy message schemas, and inputs to the
recorded-data previews and measurements come from this work. The packaged
`nav_ross` IDLs were reconstructed from the ROS message descriptions in the
converted MCAP. The authors' [Navtech high-resolution ROS repository](https://github.com/unmannedlab/navtechradar_usl)
is the reference for the original topic and encoder conventions. The checkout
includes the interfaces and example images. Obtain recordings from the dataset authors.

## Related implementations

These projects informed the design review. The native playback nodes in this
repository implement the same ROS contracts as the Python reference here.

- [Unmanned-Systems-Lab/navtech_radar_ros_sdk](https://github.com/Unmanned-Systems-Lab/navtech_radar_ros_sdk) provides an IASDK and example ROS nodes for comparison.
- [norlab-ulaval/navtech_radar_ros](https://github.com/norlab-ulaval/navtech_radar_ros) documents an earlier ROS 2 integration.
- [biomotion/navtech_projection](https://github.com/biomotion/navtech_projection) illustrates polar-to-Cartesian radar image projection.
- [utiasASRL/steam_icp](https://github.com/utiasASRL/steam_icp) provides a downstream radar odometry reference.

The pipeline uses the [ROS 2 middleware and message interfaces](https://docs.ros.org/en/lyrical/),
[NumPy](https://numpy.org/doc/stable/) and [OpenCV](https://docs.opencv.org/4.x/).
Those packages retain their respective licenses. The Apache-2.0 license at the
repository root applies to this project's code.
