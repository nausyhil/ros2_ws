import os
# import logging
# logging.root.setLevel(logging.DEBUG)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('my_nav2_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    namespace       = LaunchConfiguration('namespace', default='')
    use_namespace   = LaunchConfiguration('use_namespace', default='false')
    map_yaml_file   = LaunchConfiguration('map', default=os.path.join(bringup_dir, 'maps', 'maze.yaml'))
    params_file     = LaunchConfiguration('params', default=os.path.join(bringup_dir, 'params', 'nav2_parameters.yaml'))
    use_sim_time    = LaunchConfiguration('use_sim_time', default='true')
    autostart       = LaunchConfiguration('autostart', default='true')
    log_level       = LaunchConfiguration('log_level', default='info')
    use_composition = LaunchConfiguration('use_composition', default='false')

    # Declare arguments
    declare_args = [
        DeclareLaunchArgument('namespace',    default_value=namespace,     description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace',default_value=use_namespace, description='Apply namespace to all nodes'),
        DeclareLaunchArgument('map',          default_value=map_yaml_file, description='Full path to map file'),
        DeclareLaunchArgument('params',       default_value=params_file,   description='Full path to parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,  description='Use simulation (Gazebo/Sim) clock'),
        DeclareLaunchArgument('autostart',    default_value=autostart,     description='Automatically start nav2 stack'),
        DeclareLaunchArgument('log_level',    default_value=log_level,     description='Logging level'),
        DeclareLaunchArgument('use_composition', default_value=use_composition)
    ]

    # Parameter substitutions for Nav2 (inject use_sim_time & map)
    param_subs = {
        'use_sim_time': use_sim_time,
        'yaml_filename': LaunchConfiguration('map')
    }
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params'),
        root_key=namespace,
        param_rewrites=param_subs,
        convert_types=True
    )

    # Common remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # 1) Your C++ diff-drive simulator node
    simulator_node = Node(
        package='diff_drive_sim_cpp',
        executable='simulator',
        name='diff_drive_sim',
        output='screen',
        respawn=False,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('odom', 'odom')],
    )

    # 2) map_to_laserscan conversion
    laserscan_node = Node(
        package='map_to_laserscan_py',
        executable='map_to_laserscan',
        name='map_to_laserscan',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'scan_topic': '/scan',
            'angle_min': -1.57,
            'angle_max':  1.57,
            'range_max':  3.0
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # 3) Include the official Nav2 bringup launch
    sanitized_use_namespace = PythonExpression(["'", use_namespace, "' == 'true'"])
    sanitized_use_sim_time = PythonExpression(["'", use_sim_time, "' == 'true'"])
    sanitized_autostart = PythonExpression(["'", autostart, "' == 'true'"])
    sanitized_use_composition = PythonExpression(["'", use_composition, "' == 'true'"])

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace':    namespace,
            'use_namespace':sanitized_use_namespace,
            'map':          map_yaml_file,
            'params_file':  params_file,
            'use_sim_time': sanitized_use_sim_time,
            'autostart':    sanitized_autostart,
            'log_level':    log_level,
            'use_composition': sanitized_use_composition
        }.items()
    )

    return LaunchDescription(declare_args + [
        # ensure we see console output without buffering
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        simulator_node,
        laserscan_node,
        nav2_bringup,
    ])