import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    rviz_path = LaunchConfiguration('rviz_path').perform(context)
    map_file = LaunchConfiguration('map_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        output='screen',
        on_exit=Shutdown(),
        sigterm_timeout='600',
        sigkill_timeout='600',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
    )

    reloc_params = [config_path, {'use_sim_time': use_sim_time}]
    if map_file:
        reloc_params.append({'relocalization.map_file': map_file})

    reloc_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_relocalization',
        name='spark_lio_relocalization',
        output='screen',
        on_exit=Shutdown(),
        parameters=reloc_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        prefix='nice',
        output='screen',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return [lio_node, reloc_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('spark_fast_lio')
    default_config = os.path.join(pkg_share, 'config', 'campus_ouster.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'campus_ouster.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='automatically start rviz'),
        DeclareLaunchArgument('config_path', default_value=default_config,
                              description='Model-specific configuration'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Set true when replaying a bag with --clock'),
        DeclareLaunchArgument('rviz_path', default_value=default_rviz,
                              description='rviz file to load'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='Optional override for relocalization.map_file'),
        OpaqueFunction(function=launch_setup),
    ])
