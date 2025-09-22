from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString


def generate_launch_description():
    bringup_pkg = get_package_share_directory('nav2_bringup')
    current_pkg = get_package_share_directory('jackal_navigation')
    navigation_launch_path = PathJoinSubstitution(
        [bringup_pkg, 'launch', 'bringup_launch.py']
    )

    params_file = LaunchConfiguration('params_file')
    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            '<robot_prefix>': LaunchConfiguration('prefix'),
        },
    )

    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'autostart': LaunchConfiguration('autostart'),
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_localization': LaunchConfiguration('localization'),
        }.items()
    )

    rviz2_path = PathJoinSubstitution([
        get_package_share_directory('jackal_navigation'),
        'rviz',
        'jackal_nav.rviz'
    ])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    args = [
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='False',
            choices=['True', 'False'],
            description='Whether to start the stack in localization mode'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                current_pkg,
                'maps',
                'raw',
                'playground.yaml'
            ]),
            description='Path to map file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [current_pkg, 'config', 'nav2_params.yaml']
            ),
            description='Full path to the navigation2 config file'
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
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    # return launchDescriptionObject
    return LaunchDescription([
        *args,
        navigation_launch_cmd,
        rviz2_node
    ])
