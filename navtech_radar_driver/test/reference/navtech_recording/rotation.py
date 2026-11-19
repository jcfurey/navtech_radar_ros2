"""Construct the expected RecordedRotation from independently decoded source data."""
from array import array

from builtin_interfaces.msg import Time
from navtech_radar_msgs.msg import RecordedRotation
import numpy as np


def stamp(ns):
    sec, nano = divmod(int(ns), 10**9)
    return Time(sec=sec, nanosec=nano)


def packed(code, values, dtype):
    result = array(code)
    result.frombytes(np.asarray(values, dtype=dtype).tobytes())
    return result


def make_rotation(frame, profile, frame_id, version):
    msg = RecordedRotation()
    geom = frame.geometry
    cfg = msg.configuration
    img = msg.image
    img.header.frame_id = frame_id
    img.header.stamp = stamp(frame.stamp)
    cfg.header = img.header
    cfg.configuration_version = version
    cfg.azimuth_samples = geom.rows
    cfg.encoder_size = geom.encoder
    cfg.range_bins = geom.bins
    cfg.bin_size = round(geom.spacing*10000)
    cfg.range_resolution = geom.spacing
    cfg.range_gain = geom.gain
    cfg.range_offset = geom.offset
    cfg.expected_rotation_rate = geom.hz
    cfg.packet_rate = geom.packet_rate
    cfg.sample_bit_depth = frame.data.dtype.itemsize*8
    img.height, img.width = frame.data.shape
    img.step = img.width * frame.data.dtype.itemsize
    img.encoding = 'mono8' if cfg.sample_bit_depth == 8 else 'mono16'
    img.is_bigendian = frame.data.dtype.byteorder == '>'
    img.data = array('B', frame.data.tobytes(order='C'))
    msg.encoder_values = packed('H', frame.encoders, np.uint16)
    msg.sweep_counters = packed('H', frame.counters, np.uint16)
    msg.sensor_ns = packed('Q', frame.sensor_ns, np.uint64)
    msg.receipt_ns = packed('Q', frame.receipt_ns, np.uint64)
    msg.row_flags = packed('B', frame.flags, np.uint8)
    msg.source_format = profile
    return msg
