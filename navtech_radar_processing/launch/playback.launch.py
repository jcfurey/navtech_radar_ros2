"""Play legacy bags, recorded FFT rotations, or vendor sensor products."""
from pathlib import Path
import subprocess

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_path


def check_playback_exit(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f'recording player exited with status {event.returncode}')


def canonical_inputs(entries):
    """Map one recorded configuration/spoke pair onto the public topic names."""
    defaults = {'/navtech_radar/configuration': 'RadarConfiguration', '/navtech_radar/fft/spoke': 'FftSpoke'}
    matches = {dst: [name for name, e in entries.items()
                     if e['topic_metadata']['type'] == 'navtech_radar_msgs/msg/' + typ]
               for dst, typ in defaults.items()}
    if not all(matches.values()):
        return {}
    if all(dst in names for dst, names in matches.items()):
        return {dst: dst for dst in defaults}
    if any(len(names) != 1 for names in matches.values()):
        raise ValueError(f'ambiguous configuration/spoke pair in bag: {matches}')
    return {names[0]: dst for dst, names in matches.items()}


def start(context):
    bag = LaunchConfiguration('bag').perform(context)
    recording = LaunchConfiguration('recording').perform(context)
    profile = LaunchConfiguration('recording_format').perform(context)
    if recording and bag:
        raise ValueError('select either bag or recording')
    source_type = None
    if recording:
        recording = str(Path(recording).expanduser())
        reader = Path(get_package_prefix('navtech_radar_driver')) / 'lib/navtech_radar_driver/recording_player'
        source_type = (subprocess.check_output([str(reader), '--identify', recording], text=True).strip()
                       if profile == 'auto' else profile)
    if source_type == 'ros2':
        bag, recording = recording, ''
    file_input = bool(recording)
    recorded_products = source_type in ('ros1', 'radiate_cartesian')
    aliases = {'/radar_data/b_scan_image': '/navtech_radar/fft/display',
               '/radar_data/point_cloud': '/navtech_radar/point_cloud',
               '/radar_data/laser_scan': '/navtech_radar/scan'}
    vendor_topics = []
    canonical = {}
    bag_spokes = 0
    road_cloud = False
    if bag:
        bag = str(Path(bag).expanduser())
        info_path = Path(bag)/'metadata.yaml'
        if info_path.exists():
            info = yaml.safe_load(info_path.read_text())['rosbag2_bagfile_information']
            entries = {t['topic_metadata']['name']: t for t in info['topics_with_message_count']}
            canonical = canonical_inputs(entries)
            if canonical:
                config_topic, spoke_topic = canonical
                bag_spokes = entries[spoke_topic]['message_count']
                if not entries[config_topic]['message_count'] or not bag_spokes:
                    raise ValueError('canonical radar playback requires a recorded configuration and spokes')
            vendor_topics = [] if bag_spokes else [t for t in entries if t in aliases]
            if '/radar_data/point_cloud' in vendor_topics:
                files = info['relative_file_paths']
                if len(files) == 1 and info['storage_identifier'] == 'sqlite3':
                    db = Path(bag) / files[0]
                    if db.stat().st_size == 457621504:
                        with db.open('rb') as stream:
                            digest = subprocess.check_output(['sha256sum'], stdin=stream, text=True).split()[0]
                            road_cloud = digest == (
                                '0bce78092533a0e432ed4321a99a4f77ffffb6eecc26244332379c7b99d741a7')
                if road_cloud:
                    aliases['/radar_data/point_cloud'] = '/navtech_road_cloud_adapter/input'
    implementation = LaunchConfiguration('implementation').perform(context)
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    def param(name, kind=str):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    assembler_params = {
        'publish_point_cloud': param('cloud', bool), 'publish_scan': param('scan', bool),
        'deskew_fixed_frame': param('deskew_fixed_frame'), 'peak_selection': param('peak_selection'),
        'scan_selection': param('scan_selection'),
        'recorded_threshold_8bit': param('recorded_threshold_8bit', int),
        'recorded_cfar_offset_8bit': param('recorded_cfar_offset_8bit', float),
    }
    threads = LaunchConfiguration('detection_threads').perform(context)
    if threads:
        assembler_params['detection_threads'] = int(threads)
    stages = []
    if not (recorded_products or vendor_topics):
        if file_input:
            assembler_params.update({'recorded_input': True, 'minimum_spokes': 1})
        elif bag_spokes:
            assembler_params.update({'bag_spoke_count': bag_spokes, 'minimum_spokes': 1})
        else:
            stages.append(('navtech_radar_driver', 'legacy_bag_driver', 'LegacyBagDriver',
                           {'frame_id': param('frame_id')}))
        stages.extend([
            ('navtech_radar_processing', 'fft_sweep_assembler', 'radar::SweepAssembler', assembler_params),
            ('navtech_radar_processing', 'fft_sweep_visualizer', 'radar::SweepVisualizer',
             {'robot_frame': param('robot_frame'), 'heading_frame': param('heading_frame')}),
        ])
    common = {'use_sim_time': sim_time}
    if stages and implementation == 'cpp_composed':
        components = [
            ComposableNode(package=pkg, plugin=plugin, name='navtech_' + exe,
                           parameters=[common, params], extra_arguments=[{'use_intra_process_comms': True}])
            for pkg, exe, plugin, params in stages
        ]
        nodes = [ComposableNodeContainer(
            package='rclcpp_components', executable='component_container', name='navtech_container',
            namespace='', arguments=['--isolated', '--executor-type', 'events-cbg'], parameters=[{'thread_num': 1}],
            composable_node_descriptions=components)]
    else:
        nodes = [Node(package=pkg, executable=exe,
                      parameters=[common, params])
                 for pkg, exe, _, params in stages]
    fixed = LaunchConfiguration('frame_id').perform(context)
    if road_cloud:
        nodes.append(Node(package='navtech_radar_driver', executable='road_cloud_adapter',
                          remappings=[('~/input', aliases['/radar_data/point_cloud'])],
                          parameters=[common, {'frame_id': param('frame_id')}],
                          on_exit=check_playback_exit))
    elif vendor_topics and fixed == 'navtech':
        fixed = {'/radar_data/b_scan_image': 'b_scan_image', '/radar_data/point_cloud': 'point_cloud',
                 '/radar_data/laser_scan': 'laser_frame'}[vendor_topics[0]]
    rviz_profile = 'recorded_products.rviz' if recorded_products or vendor_topics else 'radar.rviz'
    nodes.append(
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[common], arguments=['-d', str(get_package_share_path('navtech_radar_processing')
                                                        / 'config' / rviz_profile),
                                               '-f', fixed]))
    if bag:
        topics = (list(canonical) if bag_spokes else ['/Navtech/Configuration_Topic', '/Navtech/FFTData'])
        topics = vendor_topics or [*topics, '/tf', '/tf_static']
        args = [bag, '--rate', LaunchConfiguration('rate')]
        if bag_spokes:
            args.extend(['--canonical', config_topic, spoke_topic])
        if road_cloud or bag_spokes:
            args.append('--wait-for-all-acked')
        remaps = canonical or {t: aliases[t] for t in vendor_topics}
        nodes.append(Node(package='navtech_radar_driver', executable='bag_player',
                          arguments=[*args, '--topics', *topics],
                          remappings=[(src, dst) for src, dst in remaps.items() if src != dst],
                          output='screen', on_exit=check_playback_exit))
    if recording:
        args = [recording, '--format', source_type, '--frame-id', LaunchConfiguration('frame_id'),
                '--rate', LaunchConfiguration('rate'), '--max-gap', LaunchConfiguration('max_gap'),
                '--limit', LaunchConfiguration('limit')]
        if not recorded_products:
            args.append('--wait-for-assembler')
        if LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1'):
            args.append('--clock')
        nodes.append(Node(package='navtech_radar_driver', executable='recording_player',
                          arguments=args, output='screen', on_exit=check_playback_exit))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='', description='ROS 2 bag directory, or empty for an external player.'),
        DeclareLaunchArgument('recording', default_value='', description='Public radar file or dataset sequence directory.'),
        DeclareLaunchArgument('recording_format', default_value='auto', description='Override dataset layout detection.'),
        DeclareLaunchArgument('max_gap', default_value='0', description='Cap file replay gaps in source seconds (0 preserves gaps).'),
        DeclareLaunchArgument('limit', default_value='0', description='Maximum file frames or ROS 1 messages (0 plays all).'),
        DeclareLaunchArgument('implementation', default_value='cpp_composed', choices=['cpp', 'cpp_composed']),
        DeclareLaunchArgument('detection_threads', default_value='', description='CFAR workers (default up to eight).'),
        DeclareLaunchArgument('peak_selection', default_value='cfar', choices=['cfar', 'threshold']),
        DeclareLaunchArgument('scan_selection', default_value='nearest', choices=['nearest', 'strongest']),
        DeclareLaunchArgument('recorded_threshold_8bit', default_value='24'),
        DeclareLaunchArgument('recorded_cfar_offset_8bit', default_value='12.0'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('scan', default_value='true'),
        DeclareLaunchArgument('cloud', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='navtech'),
        DeclareLaunchArgument('deskew_fixed_frame', default_value=''),
        DeclareLaunchArgument('robot_frame', default_value=''),
        DeclareLaunchArgument('heading_frame', default_value=''),
        OpaqueFunction(function=start),
    ])
