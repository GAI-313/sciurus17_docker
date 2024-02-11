#!/usr/bin/env python3
from launch import LaunchDescription # launch するために必ず必要
from launch.launch_description_sources import PythonLaunchDescriptionSource # python launch ファイルを参照するクラス
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource # XML launch ファイルを参照するクラス
from launch_ros.actions import Node # ノードを追加するためのクラス
from launch.actions import IncludeLaunchDescription # launch ファイルをインクルードするクラス
from launch.substitutions import LaunchConfiguration # 置換設定
from launch.actions import DeclareLaunchArgument # arg
from launch.conditions import IfCondition, UnlessCondition # 実行可否を指定するためのパッケージ

from ament_index_python.packages import get_package_share_directory # パッケージのパスを取得

import os # ファイル操作
import traceback

def generate_launch_description():
    try:
        print("minimum bringup 0.1")

        ld = LaunchDescription()

        # get robot model
        robot_model = os.environ['TURTLEBOT3_MODEL']
        # view the rviz setting via argument
        rviz_view = LaunchConfiguration("rviz_view")
        rviz_view_arg = DeclareLaunchArgument("rviz_view", default_value="false")
        # cam teleop arg
        cam_teleop_conf = LaunchConfiguration("cam_teleop")
        cam_teleop_conf_arg = DeclareLaunchArgument("cam_teleop", default_value="false")
        # gazebo simulation launch
        simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                        get_package_share_directory("turtlebot3_gazebo") + "/launch/turtlebot3_world.launch.py"])
        )

        # bringup launch
        bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource([
                        get_package_share_directory("turtlebot3_bringup") + "/launch/turtlebot3_state_publisher.launch.py"])
        )

        # joy control
        joy = Node(package = "joy",
                    executable = "joy_node")
        joy_teleop = Node(package = "tb3_common",
                    executable = "joy_teleop")

        # cam control
        cam_teleop = Node(package = "tb3_common",
                    executable = "cam_teleop",
                    condition=IfCondition(cam_teleop_conf))

        # rviz2
        rviz = Node(package = "rviz2",
                    executable = "rviz2",
                    arguments=["-d", os.path.join(get_package_share_directory("tb3_common"), "rviz", "minimum_%s.rviz"%robot_model)],
                    condition=IfCondition(rviz_view))

        ld.add_action(rviz_view_arg)
        ld.add_action(cam_teleop_conf_arg)
        ld.add_action(simulation)
        ld.add_action(bringup)
        ld.add_action(joy)
        ld.add_action(joy_teleop)
        ld.add_action(cam_teleop)
        ld.add_action(rviz)

        return ld

    except:
        print(traceback.print_exc())
