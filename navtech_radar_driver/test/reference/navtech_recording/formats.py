"""Decode radar rotations without ROS. Arrays retain acquisition order and FFT units."""
from dataclasses import dataclass
from decimal import Decimal
import json
from pathlib import Path
import re
import shutil
from contextlib import ExitStack
import struct
import tarfile
import tempfile
import zipfile
import zlib

import cv2
import numpy as np

from .boundary import counter_crossed_zero


@dataclass
class Geometry:
    rows: int
    encoder: int
    bins: int
    spacing: float
    hz: int = 4
    gain: float = 1.0
    offset: float = 0.0
    packet_rate: int = 0


@dataclass
class Rotation:
    geometry: Geometry
    stamp: int
    data: np.ndarray
    encoders: np.ndarray
    sensor_ns: object = ()
    counters: object = ()
    receipt_ns: object = ()
    flags: object = ()


def png(data):
    # Check IHDR before OpenCV allocates the image.
    if data[:8] != b'\x89PNG\r\n\x1a\n' or len(data) < 29:
        raise ValueError('invalid radar PNG header')
    w, h = struct.unpack_from('>II', data, 16)
    if min(w, h) < 1 or max(w, h) > 16400 or w*h > 2048*16400 or data[24:26] != b'\x08\x00':
        raise ValueError(f'unsupported radar PNG geometry or encoding: {w} x {h}')
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None or image.shape != (h, w):
        raise ValueError('could not decode the complete radar PNG')
    return image


def uniform(rows):
    return np.rint(np.arange(rows) * (5600 / rows)).astype(np.uint16)


def field(image, start, dtype):
    width = np.dtype(dtype).itemsize
    return image[:, start:start+width].copy().view(dtype).ravel()


def polar(profile, image, frame_stamp):
    if profile in ('radiate', 'mulran', 'muses'):
        image = image.T
    rows, width = image.shape
    if not 1 <= rows <= 2048 or (profile in ('radiate', 'mulran', 'muses') and rows != 400):
        raise ValueError(f'{profile}: unexpected azimuth dimension {rows}')
    if profile in ('radiate', 'mulran'):
        spacing = 0.17361 if profile == 'radiate' else 0.0596
        return Rotation(Geometry(rows, 5600, width, spacing), frame_stamp, image, uniform(rows))
    if width <= 11:
        raise ValueError(f'{profile}: missing FFT bins after the 11-byte row header')
    enc = field(image, 8, '<u2')
    flags = image[:, 10].copy()
    counters = ()
    spacing, offset = 0.0438, 0.0
    if profile == 'muses':
        seconds = field(image, 0, '<u4').astype(np.uint64)
        nsec = field(image, 4, '<u4')
        if np.any(nsec >= 10**9):
            raise ValueError('invalid MUSES nanoseconds')
        stamps = seconds * 10**9 + nsec
        offset = spacing  # ETH assigns one range increment to the first bin.
    else:
        stamps = field(image, 0, '<u8') * 1000
        if profile == 'oord':
            # The Oxford Offroad loader uses row angle. Bytes 8:10 count packets.
            counters, enc = enc, uniform(rows)
        if profile in ('oxford', 'oord'):
            spacing = 0.0432
            offset = spacing / 2  # Oxford projects the centre of bin zero here.
        if profile in ('boreas', 'boreas_rt'):
            # ASRL's radar-resolution upgrade boundary is in Unix seconds.
            spacing = 0.04381 if int(stamps[0]) > 1632182400 * 10**9 else 0.0596
            offset = -0.31
    if np.any(enc >= 5600):
        raise ValueError(f'{profile}: encoder outside a revolution')
    return Rotation(Geometry(400, 5600, width-11, spacing, offset=offset),
                    int(stamps[0]), image[:, 11:], enc, stamps, counters, flags=flags)


def ray_csv(data):
    # Decimal timestamps exceed float64 integer precision. Parse those separately.
    stamps, enc, counters, samples = [], [], [], []
    for line in data.splitlines():
        if not line.strip():
            continue
        stamp, _, angle, counter, bins = line.split(b',', 4)
        values = np.fromstring(bins.rstrip(b',').decode('ascii'), dtype=np.int32, sep=',')
        if values.size != 3360 or values.min() < 0 or values.max() > 255:
            raise ValueError('invalid MulRan FFT row')
        stamps.append(int(stamp))
        enc.append(int(angle))
        counters.append(int(counter))
        samples.append(values.astype(np.uint8))
    return Rotation(Geometry(400, 5600, 3360, 0.0596), stamps[0],
                    np.stack(samples), np.array(enc), stamps, counters)


def file_rotation(profile, name, data, stamps):
    if profile == 'mulran_ray':
        return ray_csv(data)
    stem = Path(name).stem
    stamp = stamps[stem] if profile in ('radiate', 'muses') else int(stem)
    return polar(profile, png(data), stamp)


def selected(profile, name):
    p = Path(name)
    if profile == 'mulran_ray':
        return p.suffix == '.csv' and p.stem.isdigit()
    if p.suffix != '.png' or p.name.startswith('._'):
        return False
    if profile == 'muses':
        return 'radar' in p.parts and p.stem.endswith('_radar')
    if profile == 'oxford':
        return p.stem.isdigit() and 'radar' in p.parts
    if profile == 'radiate':
        return p.stem.isdigit() and 'Navtech_Polar' in p.parts
    return p.stem.isdigit()


def timestamp_map(profile, read):
    if profile in ('radiate', 'radiate_cartesian'):
        table = 'Navtech_Cartesian.txt' if profile == 'radiate_cartesian' else 'Navtech_Polar.txt'
        text = read(table).decode()
        return {k: int(Decimal(v)*10**9) for k, v in re.findall(r'Frame:\s*(\d+)\s+Time:\s*([\d.]+)', text)}
    if profile == 'muses':
        meta = json.loads(read('meta.json'))
        return {Path(v['path_to_radar']).stem: int(v['radar_start_timestamp_us'])*1000
                for v in meta.values()}
    return {}


def radiate_cartesian_images(path):
    folder = path if path.name == 'Navtech_Cartesian' else path/'Navtech_Cartesian'
    stamps = timestamp_map('radiate_cartesian', lambda name: (folder.parent/name).read_bytes())
    for image in sorted(folder.glob('*.png'), key=lambda p: stamps[p.stem]):
        yield stamps[image.stem], png(image.read_bytes())


def dataset_rotations(path, profile):
    def chronological(name):
        stem = Path(name).stem
        return stamps[stem] if stamps else int(stem)

    if path.is_file() and path.name.endswith('.tar.gz'):
        # The public Offroad tar is shuffled. Spool only radar PNGs and sort by
        # timestamp. Streaming the compressed archive in physical order jumps time.
        with tempfile.TemporaryDirectory(prefix='navtech-oord-') as folder:
            root = Path(folder)
            with tarfile.open(path, 'r|gz') as archive:
                for member in archive:
                    if member.isfile() and selected(profile, member.name):
                        target = root/Path(member.name).name
                        with archive.extractfile(member) as src, target.open('xb') as dest:
                            shutil.copyfileobj(src, dest)
            yield from dataset_rotations(root, profile)
        return
    if path.is_file() and path.suffix.lower() == '.zip':
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            def read_meta(suffix):
                key, = [n for n in names if Path(n).name == suffix]
                return archive.read(key)
            stamps = timestamp_map(profile, read_meta)
            selected_names = [n for n in names if selected(profile, n)]
            for n in sorted(selected_names, key=chronological):
                yield file_rotation(profile, n, archive.read(n), stamps)
        return
    if not path.is_dir():
        raise ValueError(f'expected a dataset directory or archive, got {path}')
    def read_meta(name):
        for parent in (path, *path.parents[:2]):
            if (parent/name).is_file():
                return (parent/name).read_bytes()
        match, = path.rglob(name)
        return match.read_bytes()
    stamps = timestamp_map(profile, read_meta)
    if profile in ('boreas', 'boreas_rt') and (path/'radar').is_dir():
        path = path/'radar'
    names = [p for p in path.rglob('*.csv' if profile == 'mulran_ray' else '*.png')
             if selected(profile, str(p))]
    for p in sorted(names, key=chronological):
        yield file_rotation(profile, p.name, p.read_bytes(), stamps)


_SIGNATURE = bytes.fromhex('0001030307070f0f1f1f3f3f7f7ffefe')
_MAX_PACKET = 4 * 1024 * 1024


def colraw_records(stream):
    while header := stream.read(13):
        if len(header) != 13:
            raise ValueError('truncated COLRAW record header')
        size, kind, high, low = struct.unpack('<IBII', header)
        if size > _MAX_PACKET:
            raise ValueError(f'oversize COLRAW record ({size} bytes)')
        packet = stream.read(size)
        if len(packet) != size:
            raise ValueError('truncated COLRAW record')
        if kind == 255:
            decoder = zlib.decompressobj(-15)
            inner = decoder.decompress(packet, _MAX_PACKET+14)
            if not decoder.eof or decoder.unused_data or not 13 <= len(inner) <= _MAX_PACKET+13:
                raise ValueError('invalid compressed COLRAW record')
            size, kind, high, low = struct.unpack_from('<IBII', inner)
            packet = inner[13:]
        if kind not in (10, 30, 31):
            continue
        if (len(packet) != size or packet[:16] != _SIGNATURE or len(packet) < 22
                or packet[17] != kind or int.from_bytes(packet[18:22], 'big') != len(packet)-22):
            raise ValueError('COLRAW packet header or payload length disagrees with record')
        yield kind, ((high << 32) | low) * 100, packet[22:]


def cpr_video_times(stream):
    """Read v1 CPR video envelopes. Navtech's converter decodes their amplitudes."""
    while header := stream.read(16):
        if len(header) != 16:
            raise ValueError('truncated CPR header')
        signature, kind, size, sec, usec = struct.unpack('>2sHIII', header)
        if signature != b'CB' or kind >> 8 != 1 or not 16 <= size <= 16*1024*1024:
            raise ValueError('unsupported CPR envelope')
        payload = stream.read(size-16)
        if len(payload) != size-16:
            raise ValueError('truncated CPR payload')
        if kind == 0x102:
            if len(payload) < 2 or usec >= 1000000:
                raise ValueError('invalid CPR video timestamp or bearing')
            yield int.from_bytes(payload[:2], 'big'), sec*10**9 + usec*1000


def colraw_rotations(path, cpr=None):
    geometry = None
    rows = []

    def rotation():
        sensor, host, enc, counters, data = zip(*rows)
        return Rotation(geometry, sensor[0] or host[0], np.stack(data),
                        np.asarray(enc), sensor, counters, host)

    with ExitStack() as files:
        stream = files.enter_context(path.open('rb'))
        video = cpr_video_times(files.enter_context(cpr.open('rb'))) if cpr else None
        for kind, host, payload in colraw_records(stream):
            if kind == 10:
                if rows:
                    yield rotation()
                    rows.clear()
                count, bin_size, bins, encoder, mhz, rate, gain, offset = struct.unpack_from('>6Hff', payload)
                if not (0 < count <= 2048 and count <= encoder and 0 < bins <= 16384 and bin_size):
                    raise ValueError('invalid COLRAW configuration')
                geometry = Geometry(count, encoder, bins, bin_size / 10000,
                                    round(mhz/1000), gain, offset, rate)
                continue
            if geometry is None:
                raise ValueError('FFT packet precedes COLRAW configuration')
            start, counter, enc = struct.unpack_from('>3H', payload)
            sec, nsec = struct.unpack_from('<II', payload, 6)
            if video is not None:
                record = next(video, None)
                if record is None:
                    raise ValueError('more converted FFTs than CPR video records')
                angle, host = record
                delta = (enc/geometry.encoder - angle/65536 + 0.5) % 1.0 - 0.5
                if abs(delta) > 1.0/geometry.encoder:
                    raise ValueError('CPR and converted FFT bearings disagree')
                # The converter writes its own wall clock into the FFT payload.
                # CPR provides the recorder time, without a separate sensor stamp.
                sec = nsec = 0
            if not (14 <= start <= len(payload) and enc < geometry.encoder and nsec < 10**9):
                raise ValueError('invalid COLRAW FFT metadata')
            data = np.frombuffer(payload, dtype=np.uint8 if kind == 30 else '<u2', offset=start)
            if len(data) != geometry.bins:
                raise ValueError('FFT range-bin count disagrees with configuration')
            sensor = sec*10**9 + nsec
            if rows:
                prev_sensor, prev_host, prev_enc, prev_counter, prev_data = rows[-1]
                # Compare within one clock. Recorder time is useful for CPR and
                # captures with an unset radar clock.
                gap = (sensor - prev_sensor if sensor and prev_sensor else
                       host - prev_host if host and prev_host else None)
                if (enc < prev_enc - geometry.encoder/2 or data.dtype != prev_data.dtype
                        or (gap is not None and mhz and gap * mhz >= 10**12)
                        or counter_crossed_zero(prev_enc, enc, (counter - prev_counter) % 65536,
                                                geometry.rows, geometry.encoder, mhz / 1000, gap)):
                    yield rotation()
                    rows.clear()
            if len(rows) >= 2048:
                raise ValueError('COLRAW azimuth did not wrap within 2048 spokes')
            rows.append((sensor, host, enc, counter, data))
        if video is not None and next(video, None) is not None:
            raise ValueError('more CPR video records than converted FFTs')
    if rows:
        yield rotation()


def identify(path, profile='auto'):
    if not path.exists():
        raise ValueError(f'recording does not exist: {path}')
    if profile != 'auto':
        return profile
    if path.suffix.lower() in ('.colraw', '.cpr', '.bag'):
        return {'.bag': 'ros1'}.get(path.suffix.lower(), path.suffix.lower()[1:])
    label = str(path).lower().replace('-', '_')
    if path.is_dir() and path.name == 'Navtech_Cartesian' and (path.parent/'Navtech_Cartesian.txt').is_file():
        return 'radiate_cartesian'
    if path.is_dir() and ((path/'Navtech_Polar.txt').is_file() or
                          path.name == 'Navtech_Polar' and (path.parent/'Navtech_Polar.txt').is_file()):
        return 'radiate'
    if path.is_dir() and (path/'metadata.yaml').is_file():
        return 'ros2'
    if path.is_file() and path.suffix == '.zip':
        with zipfile.ZipFile(path) as z:
            names = z.namelist()
            if any(n.endswith('.bag') for n in names):
                return 'ros1'
            if any(n.endswith('metadata.yaml') for n in names):
                raise ValueError('select one extracted ROS 2 bag directory from this ZIP')
            if any(n.startswith('muses/radar/') for n in names):
                return 'muses'
            if any('/radar/' in n and n.endswith('.png') for n in names):
                return 'oxford'
            if any(n.startswith('ray/') for n in names):
                return 'mulran_ray'
            if any(n.startswith('polar/') for n in names):
                return 'mulran'
    for marker, kind in [('road_trip', 'boreas_rt'), ('boreas_rt', 'boreas_rt'),
                         ('boreas', 'boreas'), ('oxford_offroad', 'oord'),
                         ('oxford_radar', 'oxford'), ('tiny_foggy', 'radiate'),
                         ('radiate', 'radiate'), ('muses', 'muses'), ('mulran', 'mulran')]:
        if marker in label:
            if kind == 'boreas' and path.is_dir():
                radar = path/'radar' if (path/'radar').is_dir() else path
                first = next((p for p in sorted(radar.glob('*.png')) if p.stem.isdigit()), None)
                # ASRL introduced chirp flags after this UTC timestamp (microseconds).
                if first and int(first.stem) > 1733248400 * 10**6:
                    return 'boreas_rt'
            return 'mulran_ray' if kind == 'mulran' and path.name == 'ray' else kind
    if path.name == 'Navtech_CTS350-X_Radar.tar.gz':
        return 'oord'
    raise ValueError('unrecognized recording layout; select recording_format explicitly')
