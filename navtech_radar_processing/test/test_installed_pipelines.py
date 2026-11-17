"""Compare native and Python executables over DDS, including partial rotations."""
from array import array
import math

from compare_playback import compare, run_pipeline
from nav_ross.msg import HighPrecisionFFTData, nav_msg
import numpy as np
import pytest
import rclpy


@pytest.mark.parametrize(('mode', 'selection'), [('cfar', 'nearest'), ('threshold', 'strongest')])
def test_installed_pipelines(mode, selection):
    cfg = nav_msg(range_resolution=0.125, AzimuthSamples=4, EncoderSize=360,
                  BinSize=1250, RangeInBins=96, ExpectedRotationRate=4)

    def spoke(angle, tick, empty=False):
        levels = array('H', [1000] * 96)
        if not empty:
            # Adjacent equal-strength reflectors test suppression and peak budget.
            levels[24], levels[26], levels[60] = 35000, 35000, 65535
        m = HighPrecisionFFTData(angle=float(angle), azimuth=angle, size=96,
                                 data=levels, sweepCounter=tick)
        m.header.stamp.sec = 12
        m.header.stamp.nanosec = tick * 62500000
        return m

    partial = [spoke(90, 0), spoke(90, 1), spoke(270, 2, empty=True)]
    complete = [spoke(a, i + 3) for i, a in enumerate([0, 90, 180, 270])]
    sentinel = spoke(0, 7)
    # Each run launches different executables. Frozen sim time gives equal
    # host stamps so acquisition and receipt metadata can both be compared.
    with rclpy.init():
        results = [run_pipeline(impl, cfg, [partial, complete], sentinel,
                                selection, mode, ['max_points_per_spoke:=1',
                                                  'azimuth_offset_degrees:=90.0'])[0]
                   for impl in ('cpp', 'python')]
    compare(*results)
    _, scan, cloud = results[0][0]
    assert cloud.width == 2
    assert scan.ranges[0] == (3.0 if selection == 'nearest' else 7.5)
    assert math.isnan(scan.ranges[1]) and math.isinf(scan.ranges[2])
    assert scan.time_increment == 0.0
    _, scan, cloud = results[0][1]
    assert scan.time_increment == 0.0625 and cloud.width == 4
    points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
    np.testing.assert_allclose(points[0], [0.0, 7.5, 0, 65535], atol=1e-5)
