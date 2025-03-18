import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 go2_description 包的路径
    go2_description_path = get_package_share_directory('go2_description')
    urdf_file = os.path.join(go2_description_path, 'urdf', 'go2_description.urdf')
    rviz_config_file = os.path.join(go2_description_path, 'launch', 'check_joint.rviz2')

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
        joint_state_publisher_gui,
        rviz_node
    ])
