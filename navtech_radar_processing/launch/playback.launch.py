"""Run the radar pipeline against a converted TAMU bag or existing legacy topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def start(context):
    bag = LaunchConfiguration('bag').perform(context)
    suffix = '_py' if LaunchConfiguration('implementation').perform(context) == 'python' else ''
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = {'use_sim_time': sim_time}
    nodes = [
        Node(package='navtech_radar_driver', executable='legacy_bag_driver' + suffix,
             parameters=[common]),
        Node(package='navtech_radar_processing', executable='fft_sweep_assembler' + suffix,
             parameters=[common, {
                 'publish_point_cloud': ParameterValue(LaunchConfiguration('cloud'), value_type=bool),
                 'publish_scan': ParameterValue(LaunchConfiguration('scan'), value_type=bool),
                 'peak_selection': LaunchConfiguration('peak_selection'),
                 'scan_selection': LaunchConfiguration('scan_selection'),
             }]),
        Node(package='navtech_radar_processing', executable='fft_sweep_visualizer',
             parameters=[common, {'robot_frame': ParameterValue(LaunchConfiguration('robot_frame'), value_type=str),
                                  'heading_frame': ParameterValue(LaunchConfiguration('heading_frame'), value_type=str)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[common], arguments=['-d', str(get_package_share_path('navtech_radar_processing')
                                                        / 'config' / 'radar.rviz')]),
    ]
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
        DeclareLaunchArgument('implementation', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('peak_selection', default_value='cfar', choices=['cfar', 'threshold']),
        DeclareLaunchArgument('scan_selection', default_value='nearest', choices=['nearest', 'strongest']),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('scan', default_value='true'),
        DeclareLaunchArgument('cloud', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_frame', default_value=''),
        DeclareLaunchArgument('heading_frame', default_value=''),
        OpaqueFunction(function=start),
    ])
