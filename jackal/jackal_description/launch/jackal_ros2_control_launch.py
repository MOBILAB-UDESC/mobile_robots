from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def arm_ros2_setup(context, *args, **kwargs):
    """
        Set up ROS 2 control for the selected robotic arm and gripper.

        Controllers are launched in sequence:
            1. arm_controller
            2. gripper_controller (after arm_controller exits, if 'gripper' is enabled)

            args[0] : namespace
            args[1] : use_sim_time
    """
    namespace = args[0]
    use_sim_time = args[1]

    arm_name = LaunchConfiguration('arm', default='').perform(context)
    gripper_name = LaunchConfiguration('gripper', default='').perform(context)

    # Check if an arm is selected
    if not arm_name:
        return []

    # arms_bringup pkg contains config files for all supported arms + grippers
    arm_bringup_pkg_path = get_package_share_directory('arms_bringup')

    # Replace the <robot_prefix> placeholder in the ros2_control.yaml file with the arm_prefix arg.
    ros2_control_params = PathJoinSubstitution(
        [arm_bringup_pkg_path, 'config', 'ros2_control.yaml'])
    ros2_control_params = ReplaceString(
        source_file=ros2_control_params,
        replacements={'<robot_prefix>': (LaunchConfiguration('arm_prefix', default=''))},
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='joint_trajectory_controller',
        namespace=namespace,
        arguments=[f'{arm_name}_arm_controller', '--param-file', ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    if gripper_name:
        gripper_controller = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            name='gripper_controller',
            namespace=namespace,
            arguments=[f'{gripper_name}_gripper_controller', '--param-file', ros2_control_params],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        arm_to_gripper = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller,
                on_exit=[gripper_controller],
            )
        )

        return [arm_controller, arm_to_gripper]

    return [arm_controller]


def generate_launch_description():
    """Load and launch ROS 2 control nodes for the Jackal robot base (optional arm+gripper)."""
    pkg_path = get_package_share_directory('jackal_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    namespace = LaunchConfiguration('namespace', default='')

    ros2_control_params = LaunchConfiguration(
        'ros2_control_params',
        default=PathJoinSubstitution([pkg_path, 'config', 'ros2_control.yaml'])
    )

    ros2_control_params = ReplaceString(
        source_file=ros2_control_params,
        replacements={'<robot_namespace_prefix>': (LaunchConfiguration('namespace_prefix', default='')),
                      '<robot_prefix>': (LaunchConfiguration('prefix', default=''))},
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='joint_state_broadcaster',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='diff_drive_controller',
        namespace=namespace,
        arguments=[f'diff_drive_base_controller', '--param-file', ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=diff_drive_base_controller,
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_base_controller,
                on_exit=[
                    OpaqueFunction(
                        function=arm_ros2_setup,
                        args=[namespace, use_sim_time],
                    )
                ],
            )
        )
    ])
