"""Run the radar pipeline against a converted TAMU bag or existing legacy topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def start(context):
    bag = LaunchConfiguration('bag').perform(context)
    implementation = LaunchConfiguration('implementation').perform(context)
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    def param(name, kind=str):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    stages = [
        ('navtech_radar_driver', 'legacy_bag_driver', 'LegacyBagDriver',
         {'frame_id': param('frame_id')}),
        ('navtech_radar_processing', 'fft_sweep_assembler', 'radar::SweepAssembler',
         {'publish_point_cloud': param('cloud', bool), 'deskew_fixed_frame': param('deskew_fixed_frame'),
          'publish_scan': param('scan', bool), 'peak_selection': param('peak_selection'),
          'scan_selection': param('scan_selection')}),
        ('navtech_radar_processing', 'fft_sweep_visualizer', 'radar::SweepVisualizer',
         {'robot_frame': param('robot_frame'), 'heading_frame': param('heading_frame')}),
    ]
    common = {'use_sim_time': sim_time}
    if implementation == 'cpp_composed':
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
        nodes = [Node(package=pkg, executable=exe + suffix, parameters=[common, params])
                 for pkg, exe, _, params in stages]
    nodes.append(
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[common], arguments=['-d', str(get_package_share_path('navtech_radar_processing')
                                                        / 'config' / 'radar.rviz'),
                                               '-f', LaunchConfiguration('frame_id')]))
    if bag:
        nodes.append(ExecuteProcess(cmd=[
            'ros2', 'bag', 'play', bag, '--clock', '100', '--delay', '2',
            '--rate', LaunchConfiguration('rate'),
            '--topics', '/Navtech/Configuration_Topic', '/Navtech/FFTData', '/tf', '/tf_static',
        ], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='', description='ROS 2 bag directory, or empty for an external player.'),
        DeclareLaunchArgument('implementation', default_value='cpp', choices=['cpp', 'cpp_composed', 'python']),
        DeclareLaunchArgument('peak_selection', default_value='cfar', choices=['cfar', 'threshold']),
        DeclareLaunchArgument('scan_selection', default_value='nearest', choices=['nearest', 'strongest']),
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
