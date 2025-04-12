from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    param_file = PathJoinSubstitution([
        FindPackageShare('legged_control'),
        'config',
        'go2.yaml'
    ])

    return LaunchDescription([

        # 主节点：pointlio_mapping
        Node(
            package='legged_control',
            executable='legged_control',
            name='legged_control_node',
            output='both',
            parameters=[param_file]
        ),
    ])
