from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'ovcs_ros2',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'test/test.urdf'])
            }.items()
        )
    ])