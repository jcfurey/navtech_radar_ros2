"""Read native publications over DDS and compare the complete messages to the recording oracles."""
from contextlib import ExitStack
import io
import json
from pathlib import Path
import struct
import subprocess
import tarfile
import time
from zipfile import ZipFile

from ament_index_python.packages import get_package_prefix
import cv2
from navtech_radar_msgs.msg import RecordedRotation
from navtech_recording import formats, ros1
from navtech_recording.road_cloud import normalize_road_cloud
from navtech_recording.rotation import make_rotation
import numpy as np
import pytest
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from test_recording_formats import record, write_colraw_boundary_loss


def executable(name):
    return Path(get_package_prefix('navtech_radar_driver')) / 'lib/navtech_radar_driver' / name


def capture(path, profile, topics, count):
    received = {t: [] for t in topics}
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('native_recording_test')
        cleanup.callback(node.destroy_node)
        for name, typ in topics.items():
            node.create_subscription(typ, name, received[name].append, 64)
        cmd = [str(executable('recording_player')), str(path), '--format', profile,
               '--delay', '0.6', '--max-gap', '0.001', '--clock']
        if RecordedRotation in topics.values():
            cmd.append('--wait-for-assembler')
        with subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True) as child:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and sum(map(len, received.values())) < count:
                rclpy.spin_once(node, timeout_sec=0.05)
                if child.poll() not in (None, 0):
                    pytest.fail(child.stderr.read())
            assert child.wait(timeout=3) == 0, child.stderr.read()
            assert sum(map(len, received.values())) == count
    return received


def png(array):
    ok, data = cv2.imencode('.png', array)
    assert ok
    return bytes(data)


@pytest.mark.parametrize('profile', ['colraw', 'cpr', 'oxford', 'oord', 'boreas', 'boreas_rt',
                                     'radiate', 'mulran', 'mulran_ray', 'muses', 'radiate_cartesian'])
def test_recording_formats_on_native_topics(tmp_path, profile):
    root = tmp_path
    if profile in ('colraw', 'cpr'):
        path = root/'turn.colraw'
        config = struct.pack('>6Hff', 4, 438, 3, 360, 4000, 16, 1.0, -0.25)
        with path.open('wb') as out:
            out.write(record(10, config))
            for tick, enc in enumerate((0, 90, 180, 270, 0)):
                payload = struct.pack('>3H', 14, (65534+tick) % 65536, enc)
                payload += struct.pack('<II', 100, tick*62500000) + struct.pack('<3H', tick, 43210, 65535)
                packet = record(31, payload, 10000+tick)
                if tick % 2:
                    import zlib
                    blob = zlib.compress(packet, wbits=-15)
                    packet = struct.pack('<IBII', len(blob), 255, 0, 0) + blob
                out.write(packet)
        cpr = None
        if profile == 'cpr':
            cpr = path.with_suffix('.cpr')
            with cpr.open('wb') as out:
                for i in range(5):
                    out.write(struct.pack('>2sHIIIH', b'CB', 0x102, 18, 99, i*62500, (i % 4)*16384))
        frames = list(formats.colraw_rotations(path, cpr))
        path = cpr or path
    else:
        raw = np.zeros((400, 28), np.uint8)
        raw[:, 11:] = np.arange(17)
        raw[:, 10] = 255
        ticks = (np.arange(400, dtype='<u2')*14)
        raw[:, 8:10] = ticks.view('u1').reshape(-1, 2)
        stamps = 1710000000000000 + np.arange(400, dtype='<u8')*625
        raw[:, :8] = stamps.view('u1').reshape(-1, 8)
        if profile == 'muses':
            raw[:, :4] = np.frombuffer(struct.pack('<I', 1710000000), 'u1')
            raw[:, 4:8] = np.frombuffer(struct.pack('<I', 123456789), 'u1')
        image = raw.T if profile in ('radiate', 'mulran', 'muses') else raw
        if profile in ('radiate', 'radiate_cartesian'):
            folder = 'Navtech_Polar' if profile == 'radiate' else 'Navtech_Cartesian'
            (root/folder).mkdir()
            (root/(folder+'.txt')).write_text('Frame: 002 Time: 1710000000.123456789\n')
            (root/folder/'002.png').write_bytes(png(image))
            path = root if profile == 'radiate' else root/folder
        elif profile == 'oord':
            path = root/'Navtech_CTS350-X_Radar.tar.gz'
            with tarfile.open(path, 'w:gz') as tar:
                for name in ['20.png', '10.png']:
                    data = png(image)
                    info = tarfile.TarInfo(name)
                    info.size = len(data)
                    tar.addfile(info, io.BytesIO(data))
        elif profile in ('boreas', 'boreas_rt'):
            path = root/profile
            (path/'radar').mkdir(parents=True)
            (path/'radar'/'1710000000000000.png').write_bytes(png(image))
        else:
            path = root/'sample.zip'
            with ZipFile(path, 'w') as archive:
                if profile == 'muses':
                    archive.writestr('muses/radar/000_radar.png', png(image))
                    archive.writestr('meta.json', json.dumps({'s': dict(path_to_radar='000_radar.png',
                                                                        radar_start_timestamp_us="1710000000000000")}))
                elif profile == 'mulran_ray':
                    archive.writestr('ray/100.csv', '1710000000123456789,0,14,65535,'+'231,'*3360+'\n')
                else:
                    archive.writestr(('ox/radar/' if profile == 'oxford' else 'polar/')+'100.png', png(image))
        if profile == 'radiate_cartesian':
            topic = '/navtech_radar/fft/cartesian'
            msgs = capture(path, profile, {topic: Image, '/clock': Clock}, 2)
            msg, = msgs[topic]
            assert msg.encoding == 'mono8' and msg.step == image.shape[1]
            assert msg.header.stamp.nanosec == 123456789 and bytes(msg.data) == image.tobytes()
            return
        frames = list(formats.dataset_rotations(path, profile))

    key = '/navtech_radar/recording/rotation'
    msgs = capture(path, profile, {key: RecordedRotation, '/clock': Clock}, len(frames)*2)
    for got, frame in zip(msgs[key], frames, strict=True):
        expected = make_rotation(frame, profile, 'navtech', 1)
        # range_gain and range_offset are float32 on the wire.
        expected = deserialize_message(serialize_message(expected), RecordedRotation)
        for field in expected.get_fields_and_field_types():
            assert getattr(got, field) == getattr(expected, field), field
    assert [c.clock.sec*10**9+c.clock.nanosec for c in msgs['/clock']] == [f.stamp for f in frames]


@pytest.mark.parametrize('sensor_clock', [False, True])
def test_native_colraw_boundary_loss(tmp_path, sensor_clock):
    path, groups = write_colraw_boundary_loss(tmp_path, sensor_clock)
    topic = '/navtech_radar/recording/rotation'
    messages = capture(path, 'auto', {topic: RecordedRotation}, len(groups))[topic]
    for msg, ticks in zip(messages, groups, strict=True):
        assert msg.image.height == len(ticks) and msg.image.width == 1
        np.testing.assert_array_equal(np.frombuffer(msg.image.data, '<u2'), ticks)
        assert list(msg.sweep_counters) == [(65300+i) % 65536 for i in ticks]


@pytest.mark.parametrize('compression', ['none', 'BZ2', 'LZ4'])
def test_archived_ros1_products(tmp_path, compression):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    t = store.types
    h = t['std_msgs/msg/Header'](4, t['builtin_interfaces/msg/Time'](35, 27), 'radar')
    data = np.arange(32, dtype='u1')
    field = t['sensor_msgs/msg/PointField']('x', 0, 7, 1)
    samples = np.array([1.1, 3., 4., 5.], np.float32)
    image = t['sensor_msgs/msg/Image'](h, 2, 8, 'mono8', 0, 16, data)
    cloud = t['sensor_msgs/msg/PointCloud2'](h, 1, 2, [field], False, 16, 32, data, True)
    scan = t['sensor_msgs/msg/LaserScan'](h, 1., -2., -1., 0., 0.25, 1., 250., samples, samples)
    path = tmp_path/'road.bag'
    writer = Writer(path)
    if compression != 'none':
        writer.set_compression(Writer.CompressionFormat[compression])
    with writer:
        for i, (name, msg) in enumerate(zip(ros1.TOPICS, (scan, cloud, image, image))):
            conn = writer.add_connection(name, msg.__msgtype__, typestore=store)
            writer.write(conn, 80_000_000_000-i*100, store.serialize_ros1(msg, msg.__msgtype__))
    archive = tmp_path/'road.ZIP'
    with ZipFile(archive, 'w') as z:
        z.write(path, 'nested/road.bag')
    expected = list(ros1.messages(path))
    types = {'sensor_msgs/msg/Image': Image, 'sensor_msgs/msg/LaserScan': LaserScan,
             'sensor_msgs/msg/PointCloud2': PointCloud2}
    topics = {'/'+k: types[v] for k, v in ros1.TOPICS.values()}
    msgs = capture(archive, 'auto', {**topics, '/clock': Clock}, 8)
    assert [m.clock.sec*10**9 + m.clock.nanosec for m in msgs['/clock']] == [r[0] for r in expected]
    for _, name, raw in expected:
        key = '/'+ros1.TOPICS[name][0]
        assert msgs[key] == [deserialize_message(raw, topics[key])]


def test_native_road_cloud_endian_and_eof():
    import copy
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('road_endian_test')
        cleanup.callback(node.destroy_node)
        source = node.create_publisher(PointCloud2, '/navtech_road_cloud_adapter/input', 10)
        output = []
        node.create_subscription(PointCloud2, '/navtech_radar/point_cloud', output.append, 10)
        with subprocess.Popen([str(executable('road_cloud_adapter'))]) as child:
            try:
                end = time.monotonic()+8
                while not source.get_subscription_count() or not node.count_publishers('/navtech_radar/point_cloud'):
                    assert time.monotonic() < end and child.poll() is None
                    rclpy.spin_once(node, timeout_sec=0.03)
                for endian in ('<', '>'):
                    fields = [PointField(name=n, offset=i*4, datatype=7, count=2)
                              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
                    data = np.array([[103, -76, 0, 250], [0, 1, 0, 12]], endian+'f4')
                    msg = PointCloud2(height=1, width=2, fields=fields, point_step=16, row_step=32,
                                      is_bigendian=endian=='>', data=data.tobytes())
                    output.clear()
                    source.publish(msg)
                    while not output:
                        assert time.monotonic() < end
                        rclpy.spin_once(node, timeout_sec=0.03)
                    assert output[0] == normalize_road_cloud(copy.deepcopy(msg), 'navtech')
                node.destroy_publisher(source)
                assert child.wait(timeout=5) == 0
            finally:
                if child.poll() is None:
                    child.terminate()
