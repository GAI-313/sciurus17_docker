#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import traceback
import os

def generate_launch_description():
    try:
        ld = LaunchDescription()

        description_space_finder = Node(
            package='s17_vision',
            executable='space_finder',
        )

        description_space_finder = Node(
            package='s17_vision',
            executable='space_finder',
        )

        ld.add_action(description_space_finder)
        
        return ld
    except:
        traceback.print_exc()
