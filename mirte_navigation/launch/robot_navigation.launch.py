import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare  
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Get the package share directory
    pkg_mirte_navigation = get_package_share_directory('mirte_navigation')

    # # Define relative paths for the map and params file
    map_arg = DeclareLaunchArgument("map", default_value=PathJoinSubstitution([pkg_mirte_navigation, "maps", 'lab_map.yaml']))
    
    nav2_params_file = os.path.join(
        pkg_mirte_navigation,
        'params',
        'mirte_nav2_params.yaml')
    
    trees_path = os.path.join(
        get_package_share_directory('task_launcher'),
        'trees',
        )
    
    params_file = ReplaceString(
        source_file=nav2_params_file,
        replacements={'<tree_path>': (trees_path, os.path.sep, 'nav2_custom_tree.xml')})

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"
        ])),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            "use_respawn": "true",
            "params_file": params_file,
            "use_sim_time": "false",
            }.items() #added respawn = True
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
        ])),
        launch_arguments={
            "params_file": params_file,
            "use_respawn": "true", # added respawn
            "use_sim_time": "false"
        }.items()
    )

    # RViz execution
    rviz_command = ExecuteProcess(
        cmd=["rviz2", "-d", PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"
        ])],
        output="screen"
    )
    relay_topic_cmd = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/cmd_vel", "/mirte_base_controller/cmd_vel"],
        name="cmd_vel_relay"
        # output="screen",
    )
    relay_topic_odom = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/mirte_base_controller/odom", "/odom"],
        # output="screen",
        name="odom_relay"
    )
    # tf_base_footprint = 
    tf_base_frame = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_frame"],
        output="screen",
    ),Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",  "base_link", "base_footprint"],
        output="screen",
    ),
    ])

    initial_pose_node = TimerAction(
        period=2.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package='mirte_navigation',
                executable='set_initial_pose',
                name='set_initial_pose',
                output='screen'
            )
        ]
    )
    return LaunchDescription([
        map_arg,
        localization_launch,
        initial_pose_node,
        navigation_launch,
        rviz_command,
        relay_topic_cmd,
        relay_topic_odom,
        tf_base_frame,
    ])
