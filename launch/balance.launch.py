import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bobble_controllers')
    default_pid_config = os.path.join(pkg_share, 'config', 'tuning_pid.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='bobble'),
        DeclareLaunchArgument('params_file', default_value=default_pid_config,
                              description='Path to PID parameter YAML file'),

        Node(
            package='bobble_controllers',
            executable='balance_node',
            name='balance_node',
            namespace=namespace,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
