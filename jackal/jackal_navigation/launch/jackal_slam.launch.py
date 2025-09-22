from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import (
    Node,
    PushROSNamespace,
    SetRemap
)
from nav2_common.launch import ReplaceString


def generate_launch_description():
    current_pkg = get_package_share_directory('jackal_navigation')
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    slam_launch_path = PathJoinSubstitution(
        [get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py']
    )
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Handle placeholders inside slam_params_file ---
    # Placeholders (<robot_namespace>, <robot_namespace_prefix>) are replaced
    # so that SLAM can run with multiple robots in unique namespaces.
    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={
            '<robot_namespace_prefix>': [namespace, '/', prefix],
            '<robot_namespace>': [namespace, '/']
        },
        condition=UnlessCondition(
            PythonExpression([
                '"',
                namespace,
                '" == ""',
            ])  # check if namespace is empty
        )
    )
    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={
            '<robot_namespace_prefix>': [prefix],
            '<robot_namespace>': ['']
        },
        condition=IfCondition(
            PythonExpression([
                '"',
                namespace,
                '" == ""',
            ])  # check if namespace is empty
        )
    )

    start_slam_toolbox_cmd = GroupAction(
        actions=[
            PushROSNamespace(namespace),  # Push all nodes under namespace
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/map', dst='map'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_path),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )

    rviz2_path = PathJoinSubstitution([current_pkg, 'rviz', 'jackal.rviz'])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        namespace=namespace,
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix added to the jackal link and joint names'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [current_pkg, 'config', 'slam.yaml']
            ),
            description='Full path to the slam_toolbox config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    return LaunchDescription([
        *args,
        start_slam_toolbox_cmd,
        rviz2_node,
    ])
