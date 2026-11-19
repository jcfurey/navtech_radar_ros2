"""Normalize Navtech's published 2022 ROS 2 road cloud export to ROS radar XYZI."""
import hashlib

import numpy as np
from sensor_msgs.msg import PointField


def is_road_cloud_export(bag_dir, metadata):
    # Other Navtech bags use the same topic and field names with different units.
    # Match the unmodified SQLite member of the manufacturer's ROS2 Bags.zip.
    files = metadata['relative_file_paths']
    if len(files) != 1 or metadata['storage_identifier'] != 'sqlite3':
        return False
    path = bag_dir / files[0]
    if path.stat().st_size != 457621504:
        return False
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() == (
            '0bce78092533a0e432ed4321a99a4f77ffffb6eecc26244332379c7b99d741a7')


def normalize_road_cloud(msg, frame_id):
    """Convert packed clockwise bin coordinates from this export in place."""
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    expected = [(name, 4*i, PointField.FLOAT32, msg.width)
                for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    if (fields != expected or msg.height != 1 or msg.point_step != 16
            or msg.row_step != 16*msg.width or len(msg.data) != msg.row_step):
        raise ValueError('unexpected layout for the Navtech road cloud export')

    points = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4').reshape(-1, 4)
    # The paired Colossus configuration gives 0.0438 metres/bin and zero offset.
    points[:, 0] *= 0.0438
    points[:, 1] *= -0.0438
    for field in msg.fields:
        field.count = 1
    msg.header.frame_id = frame_id
    return msg
