import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 go2_description 包路径
    go2_description_path = get_package_share_directory('go2_description')
    urdf_file = os.path.join(go2_description_path, 'urdf', 'go2_description.urdf')

    # 获取 gazebo_ros 包路径
    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    # 加载 Gazebo 空白世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        )
    )

    # 静态 TF 变换 base_link -> base_footprint
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # 在 Gazebo 中生成机器人模型
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'go2_description'],
        output='screen'
    )

    # 发布 /calibrated 话题（替代 ROS 1 的 `rostopic pub`）
    fake_joint_calibration = Node(
        package='std_msgs',
        executable='pub',
        arguments=['/calibrated', 'std_msgs/msg/Bool', 'true']
    )

    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        spawn_model,
        # fake_joint_calibration
    ])
