import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rotation_matrix_to_quaternion(r):
    m = [[r[0], r[1], r[2]],
         [r[3], r[4], r[5]],
         [r[6], r[7], r[8]]]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m[2][1] - m[1][2]) * s
        qy = (m[0][2] - m[2][0]) * s
        qz = (m[1][0] - m[0][1]) * s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    rviz_path = LaunchConfiguration('rviz_path').perform(context)

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    params = cfg['/**']['ros__parameters']

    lidar_frame = params['common']['lidar_frame']
    imu_frame = params['common']['imu_frame']
    extrinsic_t = params['mapping']['extrinsic_T']
    extrinsic_r = params['mapping']['extrinsic_R']
    qx, qy, qz, qw = rotation_matrix_to_quaternion(extrinsic_r)

    lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        output='screen',
        on_exit=Shutdown(),
        parameters=[config_path],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_lidar_static_tf',
        output='screen',
        arguments=[
            '--x', str(extrinsic_t[0]),
            '--y', str(extrinsic_t[1]),
            '--z', str(extrinsic_t[2]),
            '--qx', str(qx),
            '--qy', str(qy),
            '--qz', str(qz),
            '--qw', str(qw),
            '--frame-id', imu_frame,
            '--child-frame-id', lidar_frame,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        prefix='nice',
        output='screen',
        arguments=['-d', rviz_path],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return [lio_node, static_tf_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('spark_fast_lio')
    default_config = os.path.join(pkg_share, 'config', 'robin_w.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'robin_w.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='automatically start rviz'),
        DeclareLaunchArgument('config_path', default_value=default_config,
                              description='Model-specific configuration'),
        DeclareLaunchArgument('rviz_path', default_value=default_rviz,
                              description='rviz file to load'),
        OpaqueFunction(function=launch_setup),
    ])
