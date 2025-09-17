from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution
)
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from nav2_common.launch import ReplaceString
import yaml


def setup_slam(context, *args, **kwargs):
    """Setup and launch slam_toolbox node."""
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Dynamically set frames names based on the prefix and namespace launch arguments.
    slam_params = {}
    namespace_prefix = prefix
    with open(slam_params_file, 'r') as file:
        yaml_content = yaml.safe_load(file)
        slam_params = yaml_content['slam_toolbox']['ros__parameters']
        if namespace:
            namespace_prefix = f'{namespace}/{prefix}'
        slam_params['odom_frame'] = f'{namespace_prefix}{slam_params['odom_frame']}'
        slam_params['base_frame'] = f'{namespace_prefix}{slam_params['base_frame']}'
        # map_frame must be the same (map) for each robot

    additional_params = {
        'use_lifecycle_manager': use_lifecycle_manager,
        'use_sim_time': use_sim_time
    }

    # based on https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[additional_params, slam_params],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[('/map', 'map'),
                    ('/map_metadata', 'map_metadata'),
                    ('/map_updates', 'map_updates')]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    return [start_async_slam_toolbox_node, configure_event, activate_event]


def generate_launch_description():

    slam_params_file = PathJoinSubstitution(
        [get_package_share_directory("jackal_navigation"), 'config', 'slam_parameters.yaml'])

    params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={'SPARSE_NORMAL_CHOLESKY': (LaunchConfiguration('namespace'), '/')},
    )

    args = [
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'
        ),
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
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
        ),
        DeclareLaunchArgument(
            'use_lifecycle_manager',
            default_value='false',
            description='Enable bond connection during node activation'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    ld = LaunchDescription([*args, OpaqueFunction(function=setup_slam, args=[params_file])])
    return ld
