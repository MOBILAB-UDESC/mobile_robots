from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    """Spawn the robot and optionally bridging its sensors."""
    namespace = LaunchConfiguration('namespace', default='')

    # Spawn the robot in Gazebo using the robot_description topic
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

    # Dynamically set topic names based on the 'prefix' launch argument.
    config_file = PathJoinSubstitution([
        get_package_share_directory('jackal_description'),
        'config',
        'bridge.yaml'
    ])

    config_file = ReplaceString(
        source_file=config_file,
        replacements={'<robot_prefix>': (LaunchConfiguration('prefix', default=''))},
    )

    # Create the gazebo bridge node
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
