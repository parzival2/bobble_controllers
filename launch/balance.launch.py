import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bobble_controllers')
    pid_config = os.path.join(pkg_share, 'config', 'balance_pid.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='bobble'),

        Node(
            package='bobble_controllers',
            executable='balance_node',
            name='balance_node',
            namespace=namespace,
            output='screen',
            parameters=[pid_config, {'use_sim_time': use_sim_time}],
        ),
    ])
