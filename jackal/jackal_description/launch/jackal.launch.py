from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Set up and return a list of ROS 2 launch actions for bringing up Jackal.

    This function performs the following tasks:
    - Generate the robot description using xacro with the specified args.
    - Launch the robot_state_publisher node to publish the robot's state.
    - Optionally launch the joint_state_publisher_gui node if simulation time is disabled.
    - Include the Gazebo simulation launch file if simulation time is enabled.
    - Launch the ROS 2 control nodes for the robot and any attached arm/gripper if enabled.
    - Optionally launch RViz2 if enabled.
    - Launch the EKF node for state estimation.
    - Launch a node to remap Twist messages to TwistStamped (for fast testing).

    Returns:
        list: Launch actions (nodes and included launch descriptions) ready for execution.
    """
    arm = LaunchConfiguration('arm').perform(context)
    arm_prefix = LaunchConfiguration('arm_prefix').perform(context)
    gripper = LaunchConfiguration('gripper')
    prefix = LaunchConfiguration('prefix').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_name = 'jackal'

    pkg_name = 'jackal_description'
    pkg_path = get_package_share_directory(pkg_name)

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_path, 'urdf', 'jackal.urdf.xacro']),
        ' arm:=', arm,
        ' arm_prefix:=', arm_prefix,
        ' gripper:=', gripper,
        ' name:=', robot_name,
        ' prefix:=', prefix,
        ' sim_gazebo:=', use_sim_time,
        ' use_camera:=', LaunchConfiguration('use_camera'),
        ' use_lidar:=', LaunchConfiguration('use_lidar'),
    ])

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Joint state GUI for manual testing
    joint_state_publisher_gui = Node(
        condition=UnlessCondition(use_sim_time),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    gazebo_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_path, 'launch', 'jackal_gazebo_launch.py']
        )),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'prefix': prefix,
            'robot_name': robot_name,
            'use_sim_time': use_sim_time,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y')
        }.items()
    )

    robot_controllers = PathJoinSubstitution([pkg_path, 'config', 'ros2_control.yaml'])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_path, 'launch', 'jackal_ros2_control_launch.py'])),
        launch_arguments={
            'arm_prefix': arm_prefix,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'ros2_control_params': robot_controllers,
            'prefix': prefix,
        }.items()
    )

    ekf_file = PathJoinSubstitution([pkg_path, 'config', 'ekf.yaml'])

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_path, 'launch', 'jackal_ekf_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_params_file': ekf_file,
            'prefix': prefix
        }.items()
    )

    # Twist message remapping (Twist -> TwistStamped)
    twist_remap_node = Node(
        package='jackal_description',
        executable='twist_to_twiststamped',
        output='screen',
        name='twist_remap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': f'{prefix}base_link'
        }],
    )

    rviz2_path = PathJoinSubstitution([pkg_path, 'rviz', 'jackal.rviz'])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        robot_state_node,
        joint_state_publisher_gui,
        gazebo_spawn_node,
        ros2_control_launch,
        ekf_node,
        twist_remap_node,
        rviz2_node
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'arm',
            default_value='',
            choices=[
                '',  # No arm
                'gen3_lite',
                'unitree_d1',
                'unitree_z1',
            ],
            description='Arm model to attach to Jackal'
        ),
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='arm_',
            description='Prefix added to the arm link and joint names'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='',
            choices=[
                '',  # No gripper
                'kinova_2f_lite',
                'robotiq_2f_85',
            ],
            description='Gripper model to attach to the arm'
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
            'use_camera',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use a camera'
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use a lidar'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Robot initial pose x'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Robot initial pose y'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.06',
            description='Robot initial pose z'
        ),
        DeclareLaunchArgument(
            'Y',
            default_value='0.0',
            description='Robot initial yaw (rotation around Z axis)'
        ),
    ]

    # Launch nodes and declared arguments
    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
