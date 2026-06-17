from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    DeclareLaunchArgument('prefix', default_value='explorer')


    xacro_file = PathJoinSubstitution([
        FindPackageShare('spark_fast_lio'),
        'urdf',
        'robot_sensors.urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'prefix:=', prefix,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static'),
        # ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='explorer'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='/explorer'
        ),
        robot_state_publisher_node,
    ])