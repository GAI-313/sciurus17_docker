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

        rviz_config = LaunchConfiguration(
                'rviz_config',
                default=os.path.join(
                    get_package_share_directory('s17_bringup'),
                    'rviz', 's17_bringup.rviz'
                )
            )

        description_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory(
                        'sciurus17_gazebo'
                    )+ '/launch/sciurus17_with_table.launch.py'
                ]
            ),
            launch_arguments={
                'rviz_config':rviz_config
            }.items()
        )

        description_pose_srvs = Node(
            package='s17_common',
            executable='pose_srvs',
        )

        description_set_pose_srvs = Node(
            package='s17_common',
            executable='set_pose.py',
        )

        description_moveit2_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory(
                        's17_bringup'
                    )+ '/launch/moveit_control.launch.py'
                ]
            )
        )

        ld.add_action(description_bringup)
        ld.add_action(description_pose_srvs)
        ld.add_action(description_set_pose_srvs)
        ld.add_action(description_moveit2_control)
        
        return ld
    except:
        traceback.print_exc()
