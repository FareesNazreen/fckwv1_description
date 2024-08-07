import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # RPLIDAR specific launch arguments
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition', #rplidar_node
            #name='rplidar_node',
            #output='screen',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }],
            output='screen'
        )
    ])

# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     return LaunchDescription([

#         Node(
#             package='rplidar_ros',
#             executable='rplidar_composition',
#             output='screen',
#             parameters=[{
#                 'serial_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0',
#                 'frame_id': 'laser_frame',
#                 'angle_compensate': True,
#                 'scan_mode': 'Standard'
#             }]
#         )
#     ])