"""Test display geometry and contrast without modifying the canonical FFT data."""

from array import array

from fft_sweep_visualizer import FftSweepVisualizer, SweepRenderer
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
import pytest
from rclpy.context import Context
from rclpy.serialization import deserialize_message, serialize_message


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
    original = serialize_message(sweep)
    render = renderer()
    polar, plan = render.render(sweep, config)
    np.testing.assert_array_equal(plan[12, 32], render._colors[64])  # +x: up
    np.testing.assert_array_equal(plan[32, 12], render._colors[128])  # +y: left
    np.testing.assert_array_equal(plan[52, 32], render._colors[191])  # -x: down
    np.testing.assert_array_equal(plan[32, 52], render._colors[255])  # -y: right
    assert polar.shape == (4, 33, 3)
    assert plan.shape == (65, 65, 3)
    assert serialize_message(sweep) == original


def test_missing_azimuths_are_black_and_duplicate_peaks_are_retained():
    values = np.zeros((4, 33), dtype=np.uint16)
    values[:, 20] = [10, 40, 20, 30]
    sweep, config = fixture_messages(values, [0, 0, 90, 180])
    render = renderer()
    polar, plan = render.render(sweep, config)
    np.testing.assert_array_equal(plan[12, 32], render._colors[255])
    assert not np.any(polar[3])
    assert not np.any(plan[32, 52])


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
    assert polar.shape == (90, 99, 3)
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
