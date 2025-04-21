from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('navigation')

    map_file = os.path.join(pkg_dir, 'map', 'map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(pkg_dir, 'map', 'map.yaml')
    # )

    # param_dir = LaunchConfiguration(
    #     'params_file',
    #     default=os.path.join(pkg_dir, 'config', 'params.yaml')
    # )

    # nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=map_dir,
        #     description='Full path to map file to load'),

        # DeclareLaunchArgument(
        #     'params_file',
        #     default_value=param_dir,
        #     description='Full path to param file to load'),

        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use simulation (Gazebo) clock if true'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir
        #     }.items(),
        # ),

        # # 启动地图服务（静态地图）
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[params_file, {'yaml_filename': map_file}]
        # ),

        # 启动规划器
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 启动控制器（如使用 TEB local planner）
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # 行为树导航器
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 回复行为服务器
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file]
        ),

        # Lifecycle 管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    # 'map_server',
                    'planner_server',
                    'controller_server',
                    'recoveries_server',
                    'bt_navigator'
                ]
            }]
        )
    ])
