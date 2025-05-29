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
    pkg_simulation = get_package_share_directory('tutorial_simulacao')

    sdf_file = os.path.join(pkg_simulation, 'simulation', 'robot.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()


    gz_sim =    IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
                                              launch_arguments={'gz_args': PathJoinSubstitution([
                                                                           pkg_simulation, 'simulation', 'building_robot.sdf'])}.items(),)
    

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )




    return LaunchDescription([
        gz_sim, 
        robot_state_publisher
    ])