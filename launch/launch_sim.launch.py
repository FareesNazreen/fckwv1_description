import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the package containing the launch files
    package_name = 'fckwv1_description'
    package_share_directory = get_package_share_directory(package_name)

    # Define paths to the launch files you want to include
    display_launch_file_path = os.path.join(package_share_directory, 'launch', 'display_sim.launch.py')
    gazebo_launch_file_path = os.path.join(package_share_directory, 'launch', 'gazebo.launch.py')

    # Create LaunchDescription instance
    ld = LaunchDescription()

    # Include display.launch.py
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_file_path)
    )
    ld.add_action(display_launch)

    # Include gazebo.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path)
    )
    ld.add_action(gazebo_launch)

    return ld
