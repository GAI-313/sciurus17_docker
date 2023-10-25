#!/usr/bin/env python3
from launch import LaunchDescription # launch するために必ず必要
from launch.launch_description_sources import PythonLaunchDescriptionSource # python launch ファイルを参照するクラス
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource # XML launch ファイルを参照するクラス
from launch_ros.actions import Node # ノードを追加するためのクラス
from launch.actions import IncludeLaunchDescription # launch ファイルをインクルードするクラス
from launch.substitutions import LaunchConfiguration # 置換設定
from launch.actions import DeclareLaunchArgument # arg

from ament_index_python.packages import get_package_share_directory # パッケージのパスを取得

import os # ファイル操作
import traceback

def generate_launch_description():
    try:
        print("minimum bringup 0.1")

        ld = LaunchDescription()

        # gazebo simulation launch
        simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                        get_package_share_directory("turtlebot3_gazebo") + "/launch/turtlebot3_world.launch.py"])
        )

        # bringup launch
        bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                        get_package_share_directory("turtlebot3_bringup") + "/launch/turtlebot3_state_publisher.launch.py"])
        )

        # rviz2
        rviz = Node(package = "rviz2",
                    executable = "rviz2",
                    arguments=["-d", os.path.join(
                    get_package_share_directory("tb3_common"), "rviz", "minimum.rviz")])

        ld.add_action(simulation)
        ld.add_action(bringup)
        ld.add_action(rviz)

        return ld

    except:
        print(traceback.print_exc())
