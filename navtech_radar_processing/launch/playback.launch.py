"""Play legacy bags, recorded FFT rotations, or vendor sensor products."""
from pathlib import Path

import yaml
from navtech_recording.formats import identify

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def check_playback_exit(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f'recording player exited with status {event.returncode}')


def start(context):
    bag = LaunchConfiguration('bag').perform(context)
    recording = LaunchConfiguration('recording').perform(context)
    profile = LaunchConfiguration('recording_format').perform(context)
    if recording and bag:
        raise ValueError('select either bag or recording')
    source_type = identify(Path(recording).expanduser(), profile) if recording else None
    if source_type == 'ros2':
        bag, recording = recording, ''
    file_input = bool(recording)
    recorded_products = source_type in ('ros1', 'radiate_cartesian')
    aliases = {'/radar_data/b_scan_image': '/navtech_radar/fft/display',
               '/radar_data/point_cloud': '/navtech_radar/point_cloud',
               '/radar_data/laser_scan': '/navtech_radar/scan'}
    vendor_topics = []
    if bag:
        bag = str(Path(bag).expanduser())
        info_path = Path(bag)/'metadata.yaml'
        if info_path.exists():
            info = yaml.safe_load(info_path.read_text())['rosbag2_bagfile_information']
            names = [t['topic_metadata']['name'] for t in info['topics_with_message_count']]
            vendor_topics = [t for t in names if t in aliases]
    implementation = LaunchConfiguration('implementation').perform(context)
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    def param(name, kind=str):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    assembler_params = {
        'publish_point_cloud': param('cloud', bool), 'publish_scan': param('scan', bool),
        'deskew_fixed_frame': param('deskew_fixed_frame'), 'peak_selection': param('peak_selection'),
        'scan_selection': param('scan_selection'),
    }
    stages = []
    if not (recorded_products or vendor_topics):
        if file_input:
            assembler_params.update({
                'recorded_input': True, 'minimum_spokes': 1,
                'recorded_threshold_8bit': param('recorded_threshold_8bit', int),
                'recorded_cfar_offset_8bit': param('recorded_cfar_offset_8bit', float),
            })
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
            namespace='', arguments=['--executor-type', 'multi-threaded'], parameters=[{'thread_num': 3}],
            composable_node_descriptions=components)]
    else:
        suffix = '_py' if implementation == 'python' else ''
        nodes = [Node(package=pkg, executable=exe + ('' if file_input and exe == 'fft_sweep_assembler' else suffix),
                      parameters=[common, params])
                 for pkg, exe, _, params in stages]
    fixed = LaunchConfiguration('frame_id').perform(context)
    if vendor_topics and fixed == 'navtech':
        fixed = {'/radar_data/b_scan_image': 'b_scan_image', '/radar_data/point_cloud': 'point_cloud',
                 '/radar_data/laser_scan': 'laser_frame'}[vendor_topics[0]]
    rviz_profile = 'recorded_products.rviz' if recorded_products or vendor_topics else 'radar.rviz'
    nodes.append(
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[common], arguments=['-d', str(get_package_share_path('navtech_radar_processing')
                                                        / 'config' / rviz_profile),
                                               '-f', fixed]))
    if bag:
        topics = vendor_topics or ['/Navtech/Configuration_Topic', '/Navtech/FFTData', '/tf', '/tf_static']
        cmd = ['ros2', 'bag', 'play', bag, '--clock', '100', '--delay', '2',
               '--rate', LaunchConfiguration('rate'), '--topics', *topics]
        if vendor_topics:
            cmd.extend(['--remap', *(t + ':=' + aliases[t] for t in vendor_topics)])
        nodes.append(ExecuteProcess(cmd=cmd, output='screen', on_exit=check_playback_exit))
    if recording:
        args = [recording, '--format', profile, '--frame-id', LaunchConfiguration('frame_id'),
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
        DeclareLaunchArgument('implementation', default_value='cpp', choices=['cpp', 'cpp_composed', 'python']),
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
