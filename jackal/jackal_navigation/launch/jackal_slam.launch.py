from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    prefix = LaunchConfiguration('prefix')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- Handle placeholders inside slam_params_file ---
    # Placeholders (<robot_prefix>) are replaced so can match the robot link names.
    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={
            '<robot_prefix>': prefix,
        },
    )

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    rviz2_path = PathJoinSubstitution([
        get_package_share_directory('jackal_navigation'),
        'rviz',
        'jackal_slam.rviz'
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
                [get_package_share_directory('jackal_navigation'), 'config', 'slam.yaml']
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
