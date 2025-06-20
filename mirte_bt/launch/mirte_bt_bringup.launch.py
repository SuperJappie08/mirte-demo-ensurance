from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('mirte_bt'),
    'config',
    'mirte_bt_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='mirte_bt',
            executable='mirte_arborist',
            name='mirte_arborist_node',
            prefix='gnome-terminal --',
            output='screen',
            parameters=[config_file,{}]
        ),
        Node(
        package="mirte_bt",
        executable="tree_action_client.py",
        arguments=["MDPGroup17"],
        prefix='gnome-terminal --',
        output="screen",
    )
    ])
