from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def modify_ekf_yaml(context, *args, **kwargs):
    """
        Prepare the EKF YAML configuration file by replacing the `<robot_namespace>` placeholder with the actual
        namespace for a robot and creates a launch node for the EKF.
    """

    # Get the launch params
    ekf_file = LaunchConfiguration(
        'ekf_params_file',
        default=os.path.join(
            get_package_share_directory('jackal_description'), 'config', 'ekf.yaml'
        )
    )
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Replace <robot_namespace> in the ekf.yaml file
    if namespace:
        namespace = f'{namespace}/'

    ekf_file = ReplaceString(
        source_file=ekf_file,
        replacements={'<robot_namespace>': namespace},
    )

    ekf_params = {}
    with open(ekf_file.perform(context), 'r') as file:
        yaml_content = yaml.safe_load(file)
        ekf_params = yaml_content['ekf_filter_node']['ros__parameters']

    # Create the ekf node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, **ekf_params}],
    )

    return [ekf_node]


def generate_launch_description():
    # Launch nodes
    return LaunchDescription([
        OpaqueFunction(function=modify_ekf_yaml)
    ])
