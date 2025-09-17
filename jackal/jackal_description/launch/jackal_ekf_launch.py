import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
import yaml


def setup_ekf(context, *args, **kwargs):
    """
        Prepare the EKF YAML configuration file by replacing the <robot_namespace_prefix> placeholder
        with the actual namespace/prefix for a robot and creates a launch node for the EKF.
    """
    ekf_config_path = LaunchConfiguration(
        'ekf_params_file',
        default=os.path.join(
            get_package_share_directory('jackal_description'), 'config', 'ekf.yaml'
        )
    )
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Replace <robot_namespace_prefix> in the ekf.yaml file to support multiple-robot prefixes
    ekf_config_path = ReplaceString(
        source_file=ekf_config_path,
        replacements={
            '<robot_namespace_prefix>': LaunchConfiguration('namespace_prefix', default='')
        },
    )
    ekf_params = {}
    with open(ekf_config_path.perform(context), 'r') as file:
        yaml_content = yaml.safe_load(file)
        ekf_params = yaml_content['ekf_filter_node']['ros__parameters']

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, **ekf_params}],
    )

    return [ekf_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=setup_ekf)
    ])
