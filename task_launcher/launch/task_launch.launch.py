from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='fsm',
        #     executable='fsm_node',
        #     name='fsm_node',
        #     parameters=['install/fsm/share/fsm/config/fsm_params.yaml'],
        #     output='screen'
        # ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("mirte_bt"), "launch", "mirte_bt_bringup.launch.py"
        ])),
        ),
        

        # Node(
        #     package='task_gui',
        #     executable='gui_node',
        #     name='gui_node',
        #     output='screen'
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('mirte_navigation'),
        #             'launch',
        #             'robot_navigation.launch.py'
        #         )
        #     )
        # ),

        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen',
            prefix='gnome-terminal --'
        ),

        Node(
            package='gripper_controller',
            executable='gripper_controller_node',
            name='gripper_controller_node',
            output='screen',
            prefix='gnome-terminal --'
        ),

        # Node(
        #     package='detection_3d_apples',
        #     executable='detection_node',
        #     name='detection_node',
        #     output='screen',
        #     prefix='gnome-terminal --',
        # ),

        # Node(
        #     package='proj_pts_location',
        #     executable='point_locator',
        #     name='point_locator',
        #     parameters=[{'image_transport': 'compressedDepth'}],
        #     output='screen',
        #     prefix='gnome-terminal --'
        # ),
    ])
