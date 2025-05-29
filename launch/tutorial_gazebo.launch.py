import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # Add ros_gz_sim -> Convenient launch files and executables for using Gazebo Sim with ROS
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_gazebo = get_package_share_directory('tutorial_simulacao')

    gz_sim =    IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
                                              launch_arguments={'gz_args': PathJoinSubstitution([
                                                                           pkg_gazebo, 'simulation', 'building_robot.sdf'])}.items(),)
    
    return LaunchDescription([
        gz_sim
    ])