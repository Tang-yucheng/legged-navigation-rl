import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 go2_description 包的路径
    go2_description_path = get_package_share_directory('go2_description')
    urdf_file = os.path.join(go2_description_path, 'urdf', 'go2_description.urdf')
    rviz_config_file = os.path.join(go2_description_path, 'launch', 'check_joint.rviz')
    legged_config_file = os.path.join(go2_description_path, 'config', 'joint_names_go2_description.yaml')

    # 声明 launch 参数
    user_debug = DeclareLaunchArgument(
        'user_debug', default_value='false',
        description='Enable user debug mode'
    )

    # 定义 robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read(), 'publish_frequency': 1000.0}]
    )

    # 定义 joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}]
    )

    legged_interaction_container = ComposableNodeContainer(
        name='legged_interaction_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 使用多线程容器
        composable_node_descriptions=[
            ComposableNode(
                package='go2_interaction',
                plugin='legged::Class_Legged_Interaction',
                name='legged_interaction_node',
                parameters=[legged_config_file]
                # parameters=[{'joint_names': joint_names}]
            )
        ],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args', '--log-level', 'legged_interaction_node:=INFO'],
        on_exit=Shutdown()
    )

    # legged_interaction_container_delay = TimerAction(
    #     period='1',  # 延迟时间
    #     actions=[
    #         legged_interaction_container
    #     ]
    # )

    # 定义 RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        user_debug,
        robot_state_publisher,
        # joint_state_publisher_gui,
        # legged_interaction_container_delay,
        legged_interaction_container,
        # rviz_node
    ])
