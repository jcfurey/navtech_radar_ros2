"""Test display geometry and contrast without modifying the canonical FFT data."""

from array import array
from contextlib import ExitStack
from copy import deepcopy
import math

from compare_images import ImageComparison
from fft_sweep_visualizer import FftSweepVisualizer, planar_yaw, SweepRenderer
from geometry_msgs.msg import Quaternion, TransformStamped
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message, serialize_message
from tf2_ros import TransformBroadcaster


def fixture_messages(values, encoders=None, *, bigendian=False, padding=0):
    values = np.asarray(values, dtype=np.uint16)
    height, width = values.shape
    config = RadarConfiguration(
        configuration_version=1, encoder_size=360, azimuth_samples=4,
        range_bins=width, range_resolution=0.5, sample_bit_depth=16,
    )
    config.header.frame_id = 'radar'
    sweep = FftSweep(configuration_version=1, range_resolution=0.5, sample_bit_depth=16)
    sweep.header.frame_id = 'radar'
    sweep.header.stamp.sec = 10
    sweep.image.header = sweep.header
    sweep.image.height, sweep.image.width = height, width
    sweep.image.encoding = 'mono16'
    sweep.image.is_bigendian = bigendian
    sweep.image.step = width * 2 + padding
    data = np.zeros((height, width + padding // 2), dtype='>u2' if bigendian else '<u2')
    data[:, :width] = values
    sweep.image.data = data.tobytes()
    sweep.encoder_values = encoders if encoders is not None else [0, 90, 180, 270][:height]
    return sweep, config


def renderer(**kwargs):
    options = {
        'min_range': 0.0, 'max_range': 16.0, 'cartesian_size': 65, 'image_width': 64,
        'lower_percentile': 0.0, 'upper_percentile': 100.0, 'annotate': False,
    }
    options.update(kwargs)
    return SweepRenderer(**options)


def test_plan_view_cardinal_axes_and_raw_data_preserved():
    values = np.zeros((4, 33), dtype=np.uint16)
    values[:, 20] = [10, 20, 30, 40]
    sweep, config = fixture_messages(values)
    original = deepcopy(sweep)
    render = renderer()
    polar, plan = render.render(sweep, config)
    np.testing.assert_array_equal(plan[12, 32], render._colors[64])  # +x: up
    np.testing.assert_array_equal(plan[32, 12], render._colors[255])  # +y: left
    np.testing.assert_array_equal(plan[52, 32], render._colors[191])  # -x: down
    np.testing.assert_array_equal(plan[32, 52], render._colors[128])  # -y: right
    assert polar.shape == (4, 33, 3)
    assert plan.shape == (65, 65, 3)
    assert sweep == original


def test_missing_azimuths_are_black_and_duplicate_peaks_are_retained():
    values = np.zeros((4, 33), dtype=np.uint16)
    values[:, 20] = [10, 40, 20, 30]
    sweep, config = fixture_messages(values, [0, 0, 90, 180])
    render = renderer()
    polar, plan = render.render(sweep, config)
    np.testing.assert_array_equal(plan[12, 32], render._colors[255])
    assert not np.any(polar[3])
    assert not np.any(plan[32, 12])


def test_peak_pooling_preserves_narrow_returns():
    values = np.zeros((4, 1000), dtype=np.uint16)
    values[0, 5] = 60000
    sweep, config = fixture_messages(values)
    render = renderer(max_range=500.0)
    polar, _ = render.render(sweep, config)
    assert polar.shape[1] <= 64
    np.testing.assert_array_equal(polar[0, 0], render._colors[255])


def test_big_endian_padded_input_matches_packed_little_endian():
    values = np.arange(132, dtype=np.uint16).reshape(4, 33)
    sweep, config = fixture_messages(values)
    expected = renderer().render(sweep, config)
    sweep, config = fixture_messages(values, bigendian=True, padding=6)
    actual = renderer().render(sweep, config)
    for before, after in zip(expected, actual):
        np.testing.assert_array_equal(before, after)


def test_cropped_range_masks_near_and_far_pixels():
    sweep, config = fixture_messages(np.arange(132).reshape(4, 33))
    _, plan = renderer(min_range=3.0, max_range=10.0).render(sweep, config)
    assert not np.any(plan[32, 32])
    assert not np.any(plan[0, 0])
    assert np.any(plan[32, 12])


def test_contrast_smoothing_resets_on_rewind_or_configuration_change():
    sweep, config = fixture_messages([[0, 100]] * 4)
    render = renderer()
    render.render(sweep, config)
    sweep.image.data = np.array([[0, 200]] * 4, dtype='<u2').tobytes()
    sweep.header.stamp.sec = 11
    render.render(sweep, config)
    assert render._bounds[1] == pytest.approx(120)
    sweep.header.stamp.sec = 1
    render.render(sweep, config)
    assert render._bounds[1] == pytest.approx(200)
    sweep.image.data = np.array([[0, 300]] * 4, dtype='<u2').tobytes()
    sweep.configuration_version = config.configuration_version = 2
    render.render(sweep, config)
    assert render._bounds[1] == pytest.approx(300)


def test_constant_image_is_finite_and_annotation_does_not_change_projection_size():
    sweep, config = fixture_messages(np.full((4, 33), 15000))
    polar, plan = renderer().render(sweep, config)
    assert not np.any(polar) and not np.any(plan)
    polar, plan = renderer(annotate=True).render(sweep, config)
    assert polar.shape == (154, 99, 3)
    assert plan.shape == (65, 65, 3)


@pytest.mark.parametrize('failure', ['version', 'encoding', 'length', 'step', 'encoders', 'range'])
def test_malformed_sweeps_are_rejected(failure):
    sweep, config = fixture_messages(np.zeros((4, 33)))
    if failure == 'version':
        sweep.configuration_version = 2
    elif failure == 'encoding':
        sweep.image.encoding = 'mono8'
    elif failure == 'length':
        sweep.image.data = array('B', [0])
    elif failure == 'step':
        sweep.image.step = 2
    elif failure == 'encoders':
        sweep.encoder_values = [0, 90, 180, 360]
    else:
        sweep.range_resolution = float('nan')
    with pytest.raises(ValueError):
        renderer().render(sweep, config)


@pytest.mark.parametrize('options', [
    {'min_range': -1.0}, {'max_range': float('inf')}, {'max_range': 0.0},
    {'image_width': 0}, {'cartesian_size': 8192}, {'gamma': 0.0},
    {'contrast_alpha': 0.0}, {'lower_percentile': 100.0},
    {'azimuth_offset_degrees': float('nan')},
])
def test_invalid_display_settings_fail_at_startup(options):
    with pytest.raises(ValueError):
        renderer(**options)


def test_node_publishes_valid_bgr8_images_in_namespace():
    class Sink:

        def __init__(self):
            self.messages = []

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            self.messages.append(deserialize_message(serialize_message(message), type(message)))

    context = Context()
    context.init()
    node = FftSweepVisualizer(context=context, namespace='/front')
    try:
        assert node._polar_publisher.topic_name == '/front/navtech_radar/fft/display'
        assert node._cartesian_publisher.topic_name == '/front/navtech_radar/fft/cartesian'
        node._polar_publisher = Sink()
        node._cartesian_publisher = Sink()
        sweep, config = fixture_messages(np.arange(132).reshape(4, 33))
        node._configuration_callback(config)
        node._sweep_callback(sweep)
        for publisher in (node._polar_publisher, node._cartesian_publisher):
            output = publisher.messages[0]
            assert output.encoding == 'bgr8'
            assert output.header == sweep.header
            assert output.step == output.width * 3
            assert len(output.data) == output.step * output.height
    finally:
        node.destroy_node()
        context.try_shutdown()


def test_shared_azimuth_offset_rotates_only_derived_projection():
    values = np.zeros((4, 33), dtype=np.uint16)
    values[:, 20] = [10, 20, 30, 40]
    sweep, config = fixture_messages(values)
    before = deepcopy(sweep)
    original_polar, _ = renderer().render(sweep, config)
    render = renderer(azimuth_offset_degrees=90.0)
    polar, plan = render.render(sweep, config)
    np.testing.assert_array_equal(plan[32, 12], render._colors[64])
    np.testing.assert_array_equal(plan[12, 32], render._colors[128])
    np.testing.assert_array_equal(polar, original_polar)
    assert sweep == before


def test_heading_quaternion_handles_inverted_mount_and_invalid_orientation():
    # Radar mounted upside down, with a -0.2 rad yaw in the vehicle frame.
    # Invert the mount to express the vehicle's forward vector in radar axes.
    inverse = Quaternion(x=-math.cos(0.1), y=math.sin(0.1), z=0.0, w=0.0)
    assert planar_yaw(inverse) == pytest.approx(-0.2)
    assert planar_yaw(Quaternion()) == 0.0
    assert planar_yaw(Quaternion(w=0.0)) is None
    assert planar_yaw(Quaternion(x=float('nan'), w=1.0)) is None
    assert planar_yaw(Quaternion(y=math.sqrt(0.5), w=math.sqrt(0.5))) is None


def test_heading_overlay_labels_reference_frame_and_coverage(monkeypatch):
    labels = []
    monkeypatch.setattr(SweepRenderer, '_text', staticmethod(
        lambda image, text, position, scale=0.45: labels.append(text)
    ))
    sweep, config = fixture_messages(np.arange(132).reshape(4, 33), [0, 1, 90, 180])
    renderer(annotate=True).render(
        sweep, config, robot_bearing=0.1, heading=(math.pi / 2, 'local_map'),
    )
    assert any('yaw in local_map: 90.0 deg' in label for label in labels)
    assert any('3/4 bearings' in label and '1 missing' in label and '1 duplicates' in label
               for label in labels)
    assert 'vehicle forward' in labels


def test_tf_heading_uses_sweep_stamp_and_does_not_reuse_missing_transform():
    context = Context()
    context.init()
    node = FftSweepVisualizer(context=context, parameter_overrides=[
        Parameter('robot_frame', value='base'), Parameter('heading_frame', value='map'),
    ])
    try:
        for seconds, yaw in ((10, 0.0), (12, math.pi / 2)):
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base'
            transform.header.stamp.sec = seconds
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            node._tf_buffer.set_transform(transform, 'test')
        sweep, _ = fixture_messages(np.zeros((4, 33)))
        sweep.header.stamp.sec = 11
        assert node._lookup_yaw('map', sweep.header.stamp) == pytest.approx(math.pi / 4)
        assert node._lookup_yaw('missing', sweep.header.stamp) is None
        sweep.header.stamp.sec = 0
        assert node._lookup_yaw('map', sweep.header.stamp) is None
        sweep.header.stamp.sec = 1
        assert node._lookup_yaw('map', sweep.header.stamp) is None
    finally:
        node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize('view', ['polar', 'cartesian'])
def test_installed_visualizers_partial_padded_sweeps(view):
    parameters = ['annotate:=false', 'min_range:=0.75', 'max_range:=125.0', 'image_width:=64',
                  'cartesian_size:=151', 'gamma:=0.65', 'azimuth_offset_degrees:=17.0']
    rng = np.random.default_rng(180)
    values = rng.integers(0, 65536, (4, 272), dtype=np.uint16)
    encoders = [0, 0, 45, 270]  # Two duplicate rows and a half-cell tie.
    sweep, config = fixture_messages(values, encoders, bigendian=True, padding=6)
    with rclpy.init(), ExitStack() as cleanup:
        check = ImageComparison(cleanup, parameters, views=(view,))
        check.configs.publish(config)
        check.frame(sweep, warmup=True)
        for version, stamp, divisor in [(1, 11, 3), (1, 7, 5), (2, 12, 2)]:
            sweep, _ = fixture_messages(values // divisor, encoders, bigendian=True, padding=6)
            sweep.configuration_version = version
            sweep.header.stamp.sec = stamp
            if version != config.configuration_version:
                config.range_gain, config.range_offset = 1.5, -0.25
                config.configuration_version = version
                check.configs.publish(config)
            check.frame(sweep, warmup=(version == 2))


def test_installed_visualizers_use_interpolated_tf_and_clear_missing_heading():
    frame = 'radar'
    robot = 'renderer_test_vehicle'
    sweep, config = fixture_messages(np.arange(4 * 601).reshape(4, 601))
    with rclpy.init(), ExitStack() as cleanup:
        check = ImageComparison(cleanup, [f'robot_frame:={robot}', 'heading_frame:=' + frame])
        broadcaster = TransformBroadcaster(check.node)
        transforms = []
        for sec, yaw in [(10, 0.0), (12, math.pi / 2)]:
            tf = TransformStamped()
            tf.header.frame_id = frame
            tf.header.stamp.sec = sec
            tf.child_frame_id = robot
            tf.transform.rotation.z = math.sin(yaw / 2)
            tf.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(tf)
        check.configs.publish(config)
        sweep.header.stamp.sec = 11
        yaw = math.pi / 4
        oracle = SweepRenderer()
        polar, plan = oracle.render(sweep, config, robot_bearing=yaw, heading=(yaw, frame))
        expected = {'polar': polar.tobytes(), 'cartesian': plan.tobytes()}

        def heading_received(outputs):
            if all(msg.data.tobytes() == expected[view] for (_, view), msg in outputs.items()):
                return True
            broadcaster.sendTransform(transforms)
            return False

        broadcaster.sendTransform(transforms)
        check.frame(sweep, warmup=True, ready=heading_received)
        # Before TF history and at the TF "latest" sentinel, suppress both overlays.
        for sec in (5, 0):
            sweep.header.stamp.sec = sec
            images = check.frame(sweep)
            polar, plan = oracle.render(sweep, config)
            assert images['polar'].data.tobytes() == polar.tobytes()
            assert images['cartesian'].data.tobytes() == plan.tobytes()


def test_calibrated_display_places_same_bin_at_new_metric_radius():
    levels = np.zeros((4, 33), dtype=np.uint16)
    levels[1, 10] = 500
    sweep, cfg = fixture_messages(levels)
    render = renderer()
    _, before = render.render(sweep, cfg)
    np.testing.assert_array_equal(before[32, 42], render._colors[255])  # 5 m right
    cfg.range_gain, cfg.range_offset = 2.0, 2.0
    polar, after = render.render(sweep, cfg)
    assert polar.shape == (4, 15, 3)  # bins 0..14 span 2..16 m
    np.testing.assert_array_equal(polar[1, 10], render._colors[255])
    np.testing.assert_array_equal(after[32, 56], render._colors[255])  # 12 m right
    assert not np.any(after[32, 42])


def test_polar_ticks_and_vehicle_forward_use_clockwise_encoder_rows(monkeypatch):
    labels = {}
    monkeypatch.setattr(SweepRenderer, '_text', staticmethod(
        lambda image, text, position, scale=0.45: labels.update({position: text})
    ))
    levels = np.zeros((4, 33), dtype=np.uint16)
    sweep, cfg = fixture_messages(levels)
    cfg.range_gain, cfg.range_offset = 2.0, 2.0
    polar, _ = renderer(annotate=True).render(sweep, cfg, robot_bearing=math.pi / 2)
    # Native rows 0/1/2/3 correspond to ROS 0/270/180/90 degrees.
    assert [labels[(5, row)] for row in range(57, 61)] == ['0', '270', '180', '90']
    np.testing.assert_array_equal(polar[55, 48], [255, 220, 0])
    assert any('100.00 cm/bin' in label for label in labels.values())
