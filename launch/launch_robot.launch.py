import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Import Node for launching ROS nodes

def generate_launch_description():
    # Get the path to the package containing the launch files
    package_name = 'fckwv1_description'
    package_share_directory = get_package_share_directory(package_name)
    
    # Define the share directory for 'robot_localization' configuration
    robot_localization_share_dir = get_package_share_directory('robot_localization')

    # Define paths to the launch files you want to include
    display_launch_file_path = os.path.join(package_share_directory, 'launch', 'display_robot.launch.py')
    rplidar_file_path = os.path.join(package_share_directory, 'launch', 'rplidar.launch.py')
    microros_file_path = os.path.join(package_share_directory, 'launch', 'microros.launch.py')

    # Create LaunchDescription instance
    ld = LaunchDescription()

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_file_path)
    )
    ld.add_action(display_launch)

    # Include rplidar.launch.py
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_file_path)
    )
    ld.add_action(rplidar_launch)

    # Run microros
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microros_file_path)
    )
    ld.add_action(microros_launch)

    # Add the odom_to_tf_node
    odom_to_tf_node = Node(
        package=package_name,
        executable='odom_to_tf_node',
        name='odom_to_tf_node',
        output='screen',
    )
    ld.add_action(odom_to_tf_node)

    # Add the wheel_transform_publisher node
    wheel_transform_publisher = Node(
        package=package_name,
        executable='wheel_transform_publisher',
        name='wheel_transform_publisher',
        output='screen',
    )
    ld.add_action(wheel_transform_publisher)

    # Add the robot_localization_node
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       #name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(robot_localization_share_dir, 'config', 'ekf.yaml')],
    )
    ld.add_action(robot_localization_node)
    
    return ld
