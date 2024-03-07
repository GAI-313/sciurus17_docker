#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch_ros.actions import SetParameter
import traceback
import os
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    try:
        ld = LaunchDescription()
        description_loader = RobotDescriptionLoader()

        robot_description_semantic_config = load_file(
            'sciurus17_moveit_config', 'config/sciurus17.srdf')
        robot_description_semantic = {
            'robot_description_semantic': robot_description_semantic_config}
        kinematics_yaml = load_yaml('sciurus17_moveit_config', 'config/kinematics.yaml')

        declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description=('Set true when using the gazebo simulator.')
        )
        
        description_moveit2_l_control = Node(
            package='s17_common',
            executable='moveit2_l_control',
            parameters=[{'robot_description': description_loader.load()},
                        robot_description_semantic,
                        kinematics_yaml]
        )

        description_moveit2_r_control = Node(
            package='s17_common',
            executable='moveit2_r_control',
            parameters=[{'robot_description': description_loader.load()},
                        robot_description_semantic,
                        kinematics_yaml]
        )

        ld.add_action(declare_use_sim_time)
        ld.add_action(SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')))
        ld.add_action(description_moveit2_l_control)
        ld.add_action(description_moveit2_r_control)
        
        return ld
    except:
        traceback.print_exc()
