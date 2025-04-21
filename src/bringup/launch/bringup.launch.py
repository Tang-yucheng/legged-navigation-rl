from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    livox_driver_dir = FindPackageShare('livox_ros_driver2').find('livox_ros_driver2')
    point_lio_dir = FindPackageShare('point_lio').find('point_lio')
    go2_description_dir = FindPackageShare('go2_description').find('go2_description')
    legged_control_dir = FindPackageShare('legged_control').find('legged_control')
    
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py'))
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(point_lio_dir, 'launch', 'point_lio.launch.py'))
    )
    go2_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go2_description_dir, 'launch', 'go2_rviz.launch.py'))
    )
    legged_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(legged_control_dir, 'launch', 'legged_control.launch.py'))
    )

    return LaunchDescription([
        lidar_driver_launch,
        mapping_launch,
        go2_description_launch,
        # legged_control_launch,
    ])
