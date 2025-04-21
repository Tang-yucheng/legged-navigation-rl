from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    param_file = PathJoinSubstitution([
        FindPackageShare('point_lio_unilidar'),
        'config',
        'unilidar_l1.yaml'
    ])

    rviz_file = PathJoinSubstitution([
        FindPackageShare('point_lio_unilidar'),
        'rviz_cfg',
        'loam_unilidar_display.rviz'
    ])

    return LaunchDescription([
        # 是否启动 RViz 参数
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz'),

        # 主节点：pointlio_mapping
        Node(
            package='point_lio_unilidar',
            executable='pointlio_mapping',
            name='point_lio_unilidar_node',
            output='screen',
            parameters=[
                # 加载 ROS 2 格式的参数文件（需要用 ros__parameters 包装）
                param_file,
                {
                    'use_imu_as_input': False,
                    'prop_at_freq_of_imu': True,
                    'check_satu': True,
                    'init_map_size': 10,
                    'point_filter_num': 1,
                    'space_down_sample': True,
                    'filter_size_surf': 0.1,
                    'filter_size_map': 0.1,
                    'cube_side_length': 1000.0,
                    'runtime_pos_log_enable': False
                }
            ]
        ),

        # 启动 octomap_server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'camera_init',  # 根据 pointlio 的 map frame 修改
                'sensor_model/max_range': 20.0,  # 可选：设置最大感知范围
                'resolution': 0.1,  # 地图分辨率
                'latch': True
            }],
            remappings=[
                ('/cloud_in', '/pointlio/cloud_registered')  # 将点云重映射为 pointlio 的输出
            ]
        ),

        # 启动 RViz
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_file],
                output='screen',
            )
        ], condition=IfCondition(LaunchConfiguration('rviz')))
    ])
