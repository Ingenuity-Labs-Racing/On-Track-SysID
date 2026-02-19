from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/car_state/odom',
        ),
        DeclareLaunchArgument(
            'ackermann_cmd_topic',
            default_value='/vesc/high_level/ackermann_cmd_mux/input/nav_1',
        ),
        DeclareLaunchArgument(
            'save_LUT_name',
            default_value='NUCx_on_track_pacejka',
        ),
        DeclareLaunchArgument(
            'plot_model',
            default_value='True',
        ),
        DeclareLaunchArgument(
            'racecar_version',
            default_value='SIM',
        ),
        Node(
            package='on_track_sys_id',
            executable='on_track_sys_id_node',
            name='on_track_sys_id',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'ackermann_cmd_topic': LaunchConfiguration('ackermann_cmd_topic'),
                'save_LUT_name': LaunchConfiguration('save_LUT_name'),
                'plot_model': LaunchConfiguration('plot_model'),
                'racecar_version': LaunchConfiguration('racecar_version'),
            }],
        ),
    ])
