# CTS350-X references

The [Great Outdoors dataset's sensor inventory](https://github.com/unmannedlab/the-great-outdoors-dataset#sensor-setup)
identifies the TAMU radar as a **Navtech CTS350-X**. The recorded configuration
does not contain a model or serial number; model identification comes from the
dataset authors.

Navtech's official [CTS350 technical datasheet, ITS-PM-0001 V2.2](https://navtechradar.com/wp-content/uploads/2024/12/CTS350-DATA-SHEET-V2.2.pdf)
covers CTS350-X and CTS350-X-P. A byte-for-byte copy downloaded on 2026-10-01 is
saved as [datasheets/CTS350-DATA-SHEET-V2.2.pdf](datasheets/CTS350-DATA-SHEET-V2.2.pdf).
The manufacturer's copyright and notices remain in the PDF; the repository's
code license does not apply to it.

SHA-256: `101a84b310e489b2f1de7b76f69ec5c48611164dceffd3bb814cad8d94e8e806`.

| Property | Datasheet standard configuration | Recorded TAMU configuration |
| --- | --- | --- |
| Radar frequency | 76–77 GHz | Not recorded |
| Azimuth field of view | 360° | 400 angular samples per revolution |
| Rotation rate | 4 Hz | 4 Hz |
| Azimuth sampling | 0.9° | 0.9°, encoder size 5600 |
| Range | 500 m radius; 1000 m diameter | 6000 bins, last bin at a nominal 262.76 m |
| Range spacing/resolution | 17.5 cm | 4.38 cm bin spacing (`BinSize=438`) |
| Raw representation | Not specified | 16-bit FFT levels |

This later datasheet describes a standard product configuration, not the
serial-specific settings of the April 2024 recording. Conversion uses the
bag's nominal spacing and assumes identity range calibration because the recording
omits the gain and offset. Neither angular sampling nor bin spacing alone establishes
the ability to resolve separate targets. No elevation, complex phase, or Doppler
measurement can be recovered from these FFT messages.
