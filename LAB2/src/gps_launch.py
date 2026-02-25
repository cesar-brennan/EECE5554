from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port for the RTK GPS')
    rtk_node = Node(
        package='gps_driver',
        executable='rtk_driver',
        name='rtk_driver_node',
        output='screen',
        arguments=[LaunchConfiguration('port')])
    return LaunchDescription([port_arg, rtk_node])
