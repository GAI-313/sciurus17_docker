
import os
import traceback
from launch import LaunchDescription # Launcher 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    try:
        # デフォルトコンフィグを設定
        use_sim_time = LaunchConfiguration("use_sim_time", default="true") # シミュレーション環境なのでデフォルトはtrue
        tb3_navigation_prefix = get_package_share_directory('tb3_navigation')
        turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
        cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                      turtlebot3_cartographer_prefix, 'config'))
        configuration_basename = LaunchConfiguration('configuration_basename',
                                                     default='turtlebot3_lds_2d.lua')
        resolution = LaunchConfiguration('resolution', default='0.05')
        publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

        rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                       'rviz', 'tb3_cartographer.rviz')

        return LaunchDescription([
            DeclareLaunchArgument(
                'cartographer_config_dir',
                default_value=cartographer_config_dir,
                description='Full path to config file to load'),
            DeclareLaunchArgument(
                'configuration_basename',
                default_value=configuration_basename,
                description='Name of lua file for cartographer'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                           '-configuration_basename', configuration_basename]),

            DeclareLaunchArgument(
                'resolution',
                default_value=resolution,
                description='Resolution of a grid cell in the published occupancy grid'),

            DeclareLaunchArgument(
                'publish_period_sec',
                default_value=publish_period_sec,
                description='OccupancyGrid publishing period'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([turtlebot3_cartographer_prefix, '/launch/occupancy_grid.launch.py']),
                launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                                  'publish_period_sec': publish_period_sec}.items(),
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'),
        ])
    except:
        traceback.print_exc()
