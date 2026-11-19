"""Play the standard sensor products in the Navtech road ROS 1 bag."""
from contextlib import contextmanager
from pathlib import Path
import subprocess
import tempfile
from zipfile import ZipFile


TOPICS = {
    '/Navtech/scan3': ('navtech_radar/scan', 'sensor_msgs/msg/LaserScan'),
    '/Navtech/FilteredPointcloud': ('navtech_radar/point_cloud', 'sensor_msgs/msg/PointCloud2'),
    '/Navtech/Polar': ('navtech_radar/fft/display', 'sensor_msgs/msg/Image'),
    '/Navtech/Cartesian1': ('navtech_radar/fft/cartesian', 'sensor_msgs/msg/Image'),
}


@contextmanager
def bag_path(path):
    if path.suffix.lower() != '.zip':
        yield path
        return
    with ZipFile(path) as archive:
        names = [n for n in archive.namelist() if n.endswith('.bag')]
        if len(names) != 1:
            raise ValueError('expected exactly one ROS 1 bag in the ZIP')
    # Navtech uses Deflate64. Python's zipfile cannot decompress this method.
    with tempfile.TemporaryDirectory(prefix='navtech-ros1-') as folder:
        tmp = Path(folder)/'recording.bag'
        with tmp.open('wb') as dest:
            subprocess.run(['unzip', '-p', str(path.resolve()), names[0]], stdout=dest, check=True)
        yield tmp


def messages(path):
    try:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError('ROS 1 playback requires the Python rosbags package') from exc
    store = get_typestore(Stores.ROS2_HUMBLE)
    with bag_path(path) as bag, Reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS and c.msgtype == TOPICS[c.topic][1]]
        if not conns:
            raise ValueError('no supported Navtech sensor products in ROS 1 bag')
        for c, stamp, raw in reader.messages(connections=conns):
            yield stamp, c.topic, bytes(store.ros1_to_cdr(raw, c.msgtype))
