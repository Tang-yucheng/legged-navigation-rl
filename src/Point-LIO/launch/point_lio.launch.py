from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    point_lio_dir = get_package_share_directory("point_lio")

    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Flag to launch RViz.'
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    # slam tf 发布节点
    slam_tf_publisher_node = Node(
        package='point_lio',
        executable='slam_tf_publisher',
        name='slam_tf_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # point_lio 里程计节点
    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
        arguments=['--ros-args', '--log-level', 'info']
    )

    # octomap_server 体素地图节点
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            # 'colored_map': True,
            'frame_id': 'world',  # 根据 pointlio 的 map frame 修改
            'sensor_model/max_range': 20.0,  # 可选：设置最大感知范围
            'resolution': 0.1,  # 地图分辨率
            'latch': True,
            "point_cloud_max_z": 1.5,
            "point_cloud_min_z": 0.15
        }],
        remappings=[
            ('/cloud_in', '/cloud_registered'),  # 将点云重映射为 pointlio 的输出
            ('/projected_map', '/map')
        ]
    )

    # rviz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    return LaunchDescription([
        rviz_arg,
        declare_namespace,
        declare_point_lio_cfg_dir,
        slam_tf_publisher_node,
        point_lio_node,
        octomap_server_node,
        rviz_node
    ])
