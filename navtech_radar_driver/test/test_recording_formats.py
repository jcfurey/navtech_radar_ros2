"""Check dataset conventions that cannot be inferred from the PNG dimensions."""
import io
import struct
import zlib

import numpy as np
import pytest

from navtech_recording.formats import colraw_records, cpr_video_times, identify, polar


def test_road_trip_chirp_and_oxford_offroad_counter():
    data = np.zeros((399, 27), dtype=np.uint8)
    counters = np.arange(65000, 65399, dtype='<u2')
    data[:, 8:10] = counters.view(np.uint8).reshape(-1, 2)
    data[:, 10] = 1
    data[:, 11:] = 222
    frame = polar('oord', data, 0)
    assert frame.geometry.rows == 400 and len(frame.encoders) == 399
    np.testing.assert_array_equal(frame.counters, counters)
    assert frame.encoders[0] == 0 and frame.encoders[-1] < 5600
    assert frame.data.shape == (399, 16) and np.all(frame.data == 222)
    data[:, 8:10] = 0
    data[::2, 10] = 0
    frame = polar('boreas_rt', data, 0)
    assert len(frame.data) == 399 and np.count_nonzero(frame.flags == 0) == 200


def test_muses_timestamp_words_and_range_axis():
    data = np.zeros((17, 400), np.uint8)
    data[:8, :] = np.frombuffer(struct.pack('<II', 1668504021, 843140512), np.uint8)[:, None]
    data[10] = 1
    data[11] = np.arange(400) % 256
    frame = polar('muses', data, 0)
    assert frame.stamp == 1668504021843140512
    assert frame.data.shape == (400, 6)
    np.testing.assert_array_equal(frame.data[:, 0], data[11])
    assert frame.geometry.offset == frame.geometry.spacing


def test_colraw_compression_and_truncation():
    signature = bytes.fromhex('0001030307070f0f1f1f3f3f7f7ffefe')
    payload = b'hello'
    packet = signature + struct.pack('>BBI', 1, 30, len(payload)) + payload
    raw = struct.pack('<IBII', len(packet), 30, 1, 2) + packet
    compressed = zlib.compress(raw, wbits=-15)
    outer = struct.pack('<IBII', len(compressed), 255, 0, 0) + compressed
    assert list(colraw_records(io.BytesIO(outer))) == [(30, ((1 << 32)+2)*100, payload)]
    for corrupt in [outer[:-1], outer[:6], raw[:-1]]:
        with pytest.raises(ValueError):
            list(colraw_records(io.BytesIO(corrupt)))


def test_cpr_video_uses_original_capture_time():
    header = struct.pack('>2sHIII', b'CB', 0x102, 18, 1718286144, 411725)
    assert list(cpr_video_times(io.BytesIO(header+b'\x83\xb3'))) == [(33715, 1718286144411725000)]
    with pytest.raises(ValueError):
        list(cpr_video_times(io.BytesIO(header)))


def test_oxford_bin_centres_and_boreas_sensor_upgrade():
    image = np.zeros((1, 12), np.uint8)
    assert polar('oxford', image, 0).geometry.offset == pytest.approx(0.0216)
    before = polar('boreas', image, 0).geometry
    image[:, :8] = np.frombuffer(struct.pack('<Q', 1752847243464332), dtype=np.uint8)
    after = polar('boreas_rt', image, 0).geometry
    assert before.spacing == pytest.approx(0.0596)
    assert after.spacing == pytest.approx(0.04381)
    assert before.offset == after.offset == -0.31


def test_colraw_native_depth_and_partial_eof(tmp_path):
    from navtech_recording.formats import colraw_rotations

    def record(kind, payload):
        signature = bytes.fromhex('0001030307070f0f1f1f3f3f7f7ffefe')
        packet = signature + struct.pack('>BBI', 1, kind, len(payload)) + payload
        return struct.pack('<IBII', len(packet), kind, 0, 2500) + packet

    config = struct.pack('>6Hff', 400, 438, 2, 5600, 4000, 1600, 1.0, 0.0)
    fft = struct.pack('>3H', 14, 65535, 5544) + struct.pack('<II', 0, 0)
    path = tmp_path/'partial.colraw'
    path.write_bytes(record(10, config) + record(31, fft + struct.pack('<HH', 0x0102, 0xff00)))
    frame, = colraw_rotations(path)
    assert frame.data.tolist() == [[0x0102, 0xff00]]
    assert frame.counters == (65535,) and frame.sensor_ns == (0,)
    assert frame.receipt_ns == (250000,) and frame.stamp == 250000


def test_ros1_header_and_bag_time_are_independent(tmp_path):
    from rclpy.serialization import deserialize_message
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    from sensor_msgs.msg import Image
    from navtech_recording.ros1 import messages

    store = get_typestore(Stores.ROS1_NOETIC)
    time = store.types['builtin_interfaces/msg/Time'](33, 456)
    header = store.types['std_msgs/msg/Header'](123, time, 'radar')
    typename = 'sensor_msgs/msg/Image'
    src = store.types[typename](header, 1, 2, 'mono8', 0, 2, np.array([13, 201], np.uint8))
    path = tmp_path/'road.bag'
    with Writer(path) as writer:
        conn = writer.add_connection('/Navtech/Polar', typename, typestore=store)
        writer.write(conn, 77_000_000_000, store.serialize_ros1(src, typename))
    (ns, topic, data), = messages(path)
    msg = deserialize_message(data, Image)
    assert ns == 77_000_000_000 and topic == '/Navtech/Polar'
    assert msg.header.frame_id == 'radar'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (33, 456)
    assert bytes(msg.data) == bytes([13, 201])


def test_identify_unmodified_boreas_sequence_name(tmp_path):
    folder = tmp_path/'boreas-2025-07-18-10-00'/'radar'
    folder.mkdir(parents=True)
    (folder/'1752847243464332.png').touch()
    assert identify(folder) == identify(folder.parent) == 'boreas_rt'


@pytest.mark.parametrize('endian', ['<', '>'])
def test_road_export_cloud_is_metric_and_scalar(endian):
    from navtech_recording.road_cloud import normalize_road_cloud
    from sensor_msgs.msg import PointCloud2, PointField

    values = np.array([[0, 100, 0, 77], [-50, 0, 0, 99]], dtype=endian+'f4')
    fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=2)
              for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
    m = PointCloud2(width=2, height=1, point_step=16, row_step=32, fields=fields,
                    data=values.tobytes(), is_bigendian=endian == '>', is_dense=True)
    m.header.stamp.sec = 123
    normalized = normalize_road_cloud(m, 'radar')
    assert normalized.header.frame_id == 'radar' and normalized.header.stamp.sec == 123
    assert [f.count for f in m.fields] == [1]*4
    np.testing.assert_allclose(np.frombuffer(m.data, dtype=endian+'f4').reshape(-1, 4),
                               [[0, -4.38, 0, 77], [-2.19, 0, 0, 99]], atol=1e-6)
    with pytest.raises(ValueError, match='layout'):
        normalize_road_cloud(m, 'radar')
