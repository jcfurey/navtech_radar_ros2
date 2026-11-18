# Navtech radar products and public recordings

For playback of the driver's native input, use the TAMU CTS350-X recording. For an independent example of ROS image, cloud and scan exports, use Navtech's moving-road-vehicle bags. For a different rotation rate and range-bin size, the manufacturer's MAS10 fog recordings include the original Colossus configuration and spokes. [TAMU playback validation](playback_validation.md) · [Navtech road recording][road] · [Navtech fog recordings][fog]

## Product families

Surveyed 18 November 2026 against Navtech's public product pages, technical datasheets and support archive. A family match establishes what was recorded. Variant, firmware, output mode and calibration can differ between units. The last column gives the closest public evidence found. [Manufacturer model index][models] · [Automation sensor comparison][compare]

| Application | Published hardware | Recording coverage |
| --- | --- | --- |
| ClearWay traffic, tunnels | CTS175-X, CTS175-X-P. [CTS175 datasheet][cts175] | No model-identified sensor recording located. |
| ClearWay traffic, highways and tunnels | CTS350-X, CTS350-X-P. [CTS350 datasheet][cts350] | CTS350-X in TAMU, Oxford Radar RobotCar, Oxford Offroad and RADIATE. No separate identification of the -P variant in those releases. |
| ClearWay traffic, North America | KTS350-X. The KTS sheet specifies 34–35 GHz, while CTS specifies 76–77 GHz. [KTS350 datasheet][kts] | No model-identified sensor recording located. |
| AdvanceGuard security, airports and ground surveillance | HDR134, HDR252, HDR282, HDR292, HDR311, HDR321, HDR331 and HDR351. Earlier HDR100 literature lists HDR114, HDR124 and HDR132. [HDR134][hdr134] · [HDR200][hdr200] · [HDR300][hdr300] · [HDR351][hdr351] · [Earlier HDR100 sheet][hdr100] | Navtech publishes application demonstrations. No downloadable HDR-identified sensor stream located. [AdvanceGuard][ag] |
| Robust Automation Sensors, RAS3 | The published variant table names RAS3-0504-H-M, RAS3-2004-M, RAS3-2004-X, RAS3-1510-M, RAS3-1510-X, RAS3-DEV-M and RAS3-DEV-X. [RAS3 variants][ras3] · [Family datasheet][ras3pdf] | A University of Turku thesis describes a RAS3 collection for pedestrian tracking. A public sensor-file download could not be verified. [University record][turku] |
| Robust Automation Sensors, RAS6 | The RAS6 product page names RAS6-054-E-H, RAS6-304-E-H, RAS6-DEV-E, RAS6-DEV-X and RAS6-DEV-A. A separate Navtech table also uses RAS6-054-H, RAS6-304-H, RAS6-504-X and RAS6-DEV. [RAS6 variants][ras6] · [Additional variant table][ras6other] · [Family datasheet][ras6pdf] | Thames, Boreas Road Trip, HeRCULES and MOANA. Navtech also labels its road example “CIR (RAS6)”. |
| Marine Automation Sensors, MAS10 | MAS10-1002-T in the current datasheet. Navtech explicitly identifies MAS10-1001-T in its fog collection. [MAS10 datasheet][maspdf] · [Recording details][fog] | Colossus captures in sea fog and a low-light port. The recording establishes coverage for the -1001-T. [More marine examples][marine] |
| Earlier compact automation sensor | Oxbotica and Navtech publicly described the compact CAS alongside CIR in their 2019 partnership. [Partner announcement][cas] | No CAS-identified sensor-file download located. |
| Industrial and earlier sensor lines | The archive covers I-Series, CIR-304-H, CIR-304-H-X, CIR-504-X, CIR-504, CIR-DEV, CIR-DEV-X and CIR Gen2 (CIR-F-E and CIR-F-X). Early AdvanceGuard installation literature also names W200/500 and AGS800/1600. [Industrial archive][legacy] · [CIR Gen2][cirgen2] · [Navtech installation guide][oldag] | CIR054-H in the reclaimer files, CIR-DEV in the road file and MUSES, CIR204/H in OSDaR23. Original Boreas has conflicting model documentation, detailed below. |

ClearWay and AdvanceGuard are complete systems. Navtech also documents Witness processing software, radar and local processing units, the Vertex radar interface, the integrator demonstration kit (NAV0229), and the IASDK with Colossus and ASTERIX CAT-240 interfaces. The RAS and MAS datasheets describe outputs for integration. SafeGuard collision avoidance is in the industrial archive. The joint Navtech/Oxbotica Terran360 localisation product is marked discontinued by Navtech. [Product directory][products] · [ClearWay][clearway] · [Processing units][rpu] · [Vertex][vertex] · [IASDK and credits](../THIRD_PARTY.md) · [SafeGuard][safeguard] · [Terran360 status][terran]

## Manufacturer recordings

| Collection | Downloadable material | What it can establish |
| --- | --- | --- |
| [Moving road vehicle][road] | Original `.colraw`, GPS NMEA, one ROS 1 bag and three ROS 2 SQLite bags. Dashcam and screen footage are available separately. | The original file's configuration reports `CIR-DEV`, 400 azimuth samples, 1,024 range bins and 4 Hz. The accompanying ROS 2 exports are useful for checking message conventions. |
| [River Thames at Tower Bridge][thames] | RAS6 radar video and navigation data in a Cambridge Pixel SPx `.cpr` recording, 4 Hz. | Marine motion, heading and scene context. Navtech provides a Windows converter from CPR to COLRAW. [Conversion guide][convert] · [Colossus playback][recordtools] |
| [Fog and low-light port][fog] | Two MAS10-1001-T `.colraw` files. GPS/AIS and a KML track accompany the open-sea case. Navtech reports no GPS/AIS for the port case. | Both configurations report 2 Hz, 400 azimuth samples, 7,744 range bins and an encoded bin size of 0.1551 m. Their embedded model name is `MAR292D1`. |
| [Bucket-wheel reclaimer][reclaimer] | Eight `.colraw` captures of a vertically mounted radar during machine movement and ore handling. | All eight files report model `CIR054-H` and 4 Hz. The vertical scan plane makes this a useful mounting-orientation case. |
| [Marine example directory][marine] | Inland waterways, open sea, harbours, berthing and shore installations. Each entry describes the attachments available. | The [offshore wind farm][wind] provides another named MAS10 case. A camera clip or radar-display video supplies scene context only. |

The Colossus configuration values above were read from the original files. The model strings come from the SDK's `ConfigurationData.model.name` protobuf field. Each of the eleven inspected `.colraw` files also contained a type 30 FFT spoke. [Navtech SDK and protocol](../THIRD_PARTY.md)

### What is in the manufacturer's ROS 2 ZIP

Each directory in `ROS2 Bags.zip` contains its own `metadata.yaml` and SQLite database. The three bags have different start timestamps. The counts below were checked against the message rows in each database. Extract the ZIP and point `ros2 bag info` or `ros2 bag play` at one of its three bag directories. [Original download][road]

| Topic | ROS 2 type | Messages |
| --- | --- | ---: |
| `/radar_data/b_scan_image` | `sensor_msgs/msg/Image` | 1,295 |
| `/radar_data/point_cloud` | `sensor_msgs/msg/PointCloud2` | 1,295 |
| `/radar_data/laser_scan` | `sensor_msgs/msg/LaserScan` | 1,850 |

The ROS 1 bag contains `/Navtech/Cartesian1` and `/Navtech/Polar` images, `/Navtech/FilteredPointcloud` and `/Navtech/scan3`. These are standard ROS 1 sensor message types. [Original download][road]

The TAMU adapter consumes `nav_ross/msg/nav_msg` and `nav_ross/msg/HighPrecisionFFTData`. The playback launch also reads the original Colossus recordings and maps the manufacturer's ROS 1 and ROS 2 products to the driver topics, including correction of the verified ROS 2 road cloud's units, handedness and scalar field counts. Use the original FFT capture for tests requiring the underlying intensity measurements. Quantitative comparison across the three bags requires time alignment and matching threshold, frame and range conventions. [Recording playback guide](recording-playback.md) · [Driver interfaces](../README.md#bag-playback)

## Research recordings

| Source | Sensor and accessible data | Driver or analysis use |
| --- | --- | --- |
| [The Great Outdoors Dataset][tamu] | Texas A&M and DEVCOM ARL. CTS350-X, vehicle-mounted. The converted April 2024 route used here contains configuration, 16-bit FFT spokes and recorded TF. | Native adapter, acquisition timing, heading, planar exports and performance regression. [Recorded configuration](cts350x.md) |
| [Oxford Radar RobotCar][ox] | CTS350-X. Oxford publishes [multisensor sample ZIPs][oxsamples] and a registered download workflow for full traversals. The tiny sample contains 11 radar images. | 8-bit range–azimuth PNG. Each row starts with 8 timestamp bytes, 2 encoder bytes and a validity flag. Exclude those fields from intensities. |
| [Oxford Offroad Radar Dataset][oord] | CTS350-X on Scottish off-road routes. Radar and GPS are separate downloads for each traversal. [Paper][oordpaper] | The first Bellmouth radar archive contains 7,097 PNGs. The first frame is 7,547 pixels wide by 400 high. |
| [RADIATE][radiate] | Heriot-Watt's CTS350-X adverse-weather dataset. The [authors' tiny fog sample][tinyfog] includes 18 polar and 18 Cartesian radar frames and timestamp files. The project page describes access to the full sequences. | The sample stores polar PNGs 400 pixels wide by 576 high and Cartesian PNGs 1,152 by 1,152. This provides both representations for checking a projection. |
| [Boreas][boreas] | University of Toronto ASRL. Public anonymous S3 scans and calibration. The original paper calls the sensor **CIR304-H** and the data guide calls it **CIR204-H**. [Paper][bpaper] · [Data guide][bguide] | Use timestamped polar PNGs and the calibration for the sequence. Keep the hardware variant unresolved when citing a scan from this release. |
| [Boreas Road Trip][brt] | ASRL's separate RAS6 collection. The data guide maps the first ten columns to per-azimuth timestamp and encoder, and the eleventh to chirp direction. | Doppler-enabled radar sequences. Treating that eleventh byte as an Oxford validity flag would discard or misclassify real measurements. |
| [MulRan][mulran] and [KAIST 2019 Radar–LiDAR][kaist] | The MulRan authors provide a public **ParkingLot** sample with polar PNGs, per-ray CSVs, timestamps and reference poses. Full MulRan sequences use their request form. KAIST 2019 is the earlier release with vehicle trajectory and reference lidar. | Use the ray timing and polar data for odometry or scan-assembly tests. The ParkingLot download page does not establish an exact model suffix. |
| [MUSES][muses] | ETH Zürich's paper identifies a CIR-DEV. The standalone [radar train/validation/test ZIP][musedl] contains 2,500 radar PNGs and ETH's non-commercial data license. [Sensor description][musepaper] | An inspected frame is 400 pixels wide by 7,547 high. Its axes run the opposite way to the inspected Oxford Offroad frame. The releases require different image readers. |
| [OSDaR23][osdar] | German railway collection. The authors identify **CIR204/H** and describe 8-bit Cartesian bird's-eye PNGs at 4 Hz. [Sensor table][osdarpaper] | Rail-scene and display comparison. The released radar representation lacks the original per-spoke intensities and encoder timing. |
| [HeRCULES][hercules] | The SNU team identifies a spinning RAS6 and a separate 4D radar. Sequence access uses the [authors' request form][hercdl]. | Select the spinning-radar stream for range–azimuth tests. The two instruments measure different quantities. |
| [MOANA][moana] | Maritime W-band **RAS6-DEV-A** in the port sequences and **RAS6-DEV-X** in the island sequences, alongside a separate X-band radar. [Sequence access][moanadl] | Marine polar images, navigation and extrinsics. Keep the two radar bands and the two RAS6 variants identifiable. |
| [Oxford Radar Doppler Dataset][doppler] | Oxford, Oxa and Navtech describe a **prototype** CTS350-X collection near Culham. Navtech's directory identifies 25 km of Doppler data. A working public sensor-file link was not verified. [Manufacturer listing][customer] | Reference for Doppler-specific acquisition. The standard CTS350-X recordings above provide intensity alone. |

Recordings and previews retain their publishers' terms. Use the citation and access instructions on each source page when redistributing data or publishing results. The manufacturer's [example directory][examples] and [customer dataset directory][customer] provide further references.

[models]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/2563178497
[compare]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/3104669702
[cts175]: https://navtechradar.com/wp-content/uploads/2026/10/CTS175-TECHNICAL-DATA-SHEET-A4-V2.0-1.pdf
[cts350]: https://navtechradar.com/wp-content/uploads/2026/08/CTS350-TECHNICAL-DATA-SHEET-A4-V3.0.pdf
[kts]: https://navtechradar.com/wp-content/uploads/2026/08/KTS-TECHNICAL-DATA-SHEET-V3.0.pdf
[hdr134]: https://navtechradar.com/wp-content/uploads/2026/08/HDR134-TECHNICAL-DATA-SHEET-A4-V4.0.pdf
[hdr200]: https://navtechradar.com/wp-content/uploads/2026/10/HDR200-TECHNICAL-DATA-SHEET-A4-V4.0.pdf
[hdr300]: https://navtechradar.com/wp-content/uploads/2026/10/HDR300-TECHNICAL-DATA-SHEET-A4-V4.0.pdf
[hdr351]: https://navtechradar.com/wp-content/uploads/2026/10/HDR351-TECHNICAL-DATA-SHEET-A4-4.0.pdf
[hdr100]: https://navtechradar.com/wp-content/uploads/2024/03/HDR100-Series-datasheet.pdf
[ag]: https://navtechradar.com/radar-solutions/radar-for-security-solutions/advanceguard-for-security/
[ras3]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2573172737
[ras3pdf]: https://navtechradar.com/wp-content/uploads/2026/08/RAS3-TECHNICAL-DATA-SHEET-A4-V2.0-1.pdf
[turku]: https://www.utupub.fi/items/efffa6b6-669a-47b8-a902-bedb68c18f1f
[ras6]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2572681217
[ras6other]: https://navtechradar.atlassian.net/wiki/download/attachments/2573172737/RAS6%20Variant%20Table.png
[ras6pdf]: https://navtechradar.com/wp-content/uploads/2026/08/RAS6-TECHNICAL-DATA-SHEET-A4-V2.0.pdf
[maspdf]: https://navtechradar.com/wp-content/uploads/2026/08/MAS10-TECHNICAL-DATA-SHEET-A4-V2.0.pdf
[legacy]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2323906595
[cas]: https://oxa.tech/news-and-insights/oxbotica-partners-with-navtech/
[cirgen2]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2431320084
[oldag]: https://fccid.io/S7Y-AGS1600-001/User-Manual/Hardware-installation-guide-1931965.pdf
[products]: https://navtechradar.atlassian.net/wiki/spaces/PROD/overview
[clearway]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/8355844
[rpu]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/1063223301
[vertex]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/29294899
[safeguard]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2061009383
[terran]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/1718485029
[road]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2061009475
[thames]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2794651653
[convert]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/2359459855
[recordtools]: https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/794558854
[fog]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/4055203841
[reclaimer]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2487058452
[marine]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2402287656
[wind]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/3180134434
[tamu]: https://github.com/unmannedlab/the-great-outdoors-dataset
[ox]: https://oxford-robotics-institute.github.io/radar-robotcar-dataset/documentation
[oxsamples]: https://oxford-robotics-institute.github.io/radar-robotcar-dataset/downloads
[oord]: https://oxford-robotics-institute.github.io/oord-dataset/
[oordpaper]: https://arxiv.org/abs/2403.02845
[radiate]: https://pro.hw.ac.uk/radiate/doc/dataset/
[tinyfog]: https://github.com/marcelsheeny/radiate_sdk/tree/b1d5de559784b3aa57bea99b00c13d495d79e846/data/radiate/tiny_foggy
[boreas]: https://www.boreas.utias.utoronto.ca/
[bpaper]: https://arxiv.org/abs/2203.10168
[bguide]: https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md
[brt]: https://github.com/utiasASRL/pyboreas/blob/master/DATA_RT_REFERENCE.md
[mulran]: https://sites.google.com/view/mulran-pr/download
[kaist]: https://sites.google.com/view/dgbicra2019-radar-lidar/
[muses]: https://github.com/timbroed/MUSES
[musedl]: https://muses.ethz.ch/MUSES_packages/
[musepaper]: https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/07626.pdf
[osdar]: https://doi.org/10.57806/9mv146r0
[osdarpaper]: https://arxiv.org/abs/2305.03001
[hercules]: https://sites.google.com/view/herculesdataset/system
[hercdl]: https://sites.google.com/view/herculesdataset/download
[moana]: https://arxiv.org/pdf/2412.03887
[moanadl]: https://sites.google.com/view/rpmmoana/download
[doppler]: https://ora.ox.ac.uk/objects/uuid:615d5176-4a75-4dad-9c94-b01b8fbad176
[examples]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2060878103
[customer]: https://navtechradar.atlassian.net/wiki/spaces/IA/pages/2323808373
