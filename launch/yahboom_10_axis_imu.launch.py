from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')
    use_euler_orientation_fallback = LaunchConfiguration('use_euler_orientation_fallback')
    output_rate_hz = LaunchConfiguration('output_rate_hz')
    save_configuration = LaunchConfiguration('save_configuration')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/imu_usb'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('use_euler_orientation_fallback', default_value='true'),
        DeclareLaunchArgument('output_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('save_configuration', default_value='false'),
        Node(
            package='yahboom_10_axis_imu',
            executable='yahboom_10_axis_imu_node',
            name='yahboom_10_axis_imu_node',
            output='screen',
            parameters=[{
                'port': port,
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'frame_id': frame_id,
                'use_euler_orientation_fallback': ParameterValue(
                    use_euler_orientation_fallback,
                    value_type=bool,
                ),
                'output_rate_hz': ParameterValue(output_rate_hz, value_type=float),
                'save_configuration': ParameterValue(save_configuration, value_type=bool),
            }],
        ),
    ])
