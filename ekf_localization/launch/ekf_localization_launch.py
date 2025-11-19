from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='src/ekf_localization/config/params.yaml',
            description='Full path to the parameter file to load'),

        Node(
            package='ekf_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])

