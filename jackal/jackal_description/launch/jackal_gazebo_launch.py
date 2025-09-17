from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    """Spawn the robot and optionally bridging its sensors."""
    namespace = LaunchConfiguration('namespace', default='')

    # Spawn the robot in Gazebo and name it according to the robot_name launch
    # argument got from jackal.launch.py
    gazebo_spawn_node = Node(
        package='ros_gz_sim', executable='create', output='screen', namespace=namespace,
        arguments=['-topic', 'robot_description',
                   '-name', LaunchConfiguration('robot_name', default='jackal'),
                   '-allow_renaming', 'true',
                   '-x', LaunchConfiguration('x', default='0.0'),
                   '-y', LaunchConfiguration('y', default='0.0'),
                   '-z', LaunchConfiguration('z', default='0.06'),
                   '-Y', LaunchConfiguration('Y', default='0.0'),],
    )

    # Dynamically set topic names based on the prefix and namespace launch arguments.
    config_file = PathJoinSubstitution([
        get_package_share_directory('jackal_description'),
        'config',
        'bridge.yaml'
    ])
    config_file = ReplaceString(
        condition=UnlessCondition(PythonExpression(["'", namespace, "' != ''"])),
        source_file=config_file,
        replacements={'<robot_namespace_prefix>': (LaunchConfiguration('namespace_prefix', default='')),
                      '<robot_namespace>': namespace},
    )
    config_file = ReplaceString(
        condition=IfCondition(PythonExpression(["'", namespace, "' != ''"])),
        source_file=config_file,
        replacements={'<robot_namespace_prefix>': (LaunchConfiguration('namespace_prefix', default='')),
                      '<robot_namespace>': (namespace, '/')},
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        namespace=namespace,
        arguments=[
            '--ros-args',
            '-p',
            ['config_file:=', config_file]
        ],
    )

    return LaunchDescription([
        gazebo_spawn_node,
        gazebo_bridge_node,
    ])
