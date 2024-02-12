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
        ld = LaunchDescription()
        # デフォルトコンフィグを設定
        use_sim_time = LaunchConfiguration("use_sim_time", default="true") # シミュレーション環境なのでデフォルトはtrue
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                os.environ['TURTLEBOT3_MODEL'] + '.yaml'))
        rviz_config_dir = os.path.join(get_package_share_directory('tb3_navigation'),
                                       'rviz', 'tb3_navigation.rviz')

        # get nav2 prefix
        nav2_prefix = get_package_share_directory('nav2_bringup')
        default_map_path = os.path.join(get_package_share_directory('tb3_navigation'),
                                                    'map', os.environ['WORLD'] + '_map.yaml')

        print(default_map_path)
        
        #map = LaunchConfiguration("map", default=default_map_path)
        map = LaunchConfiguration("map")

        # add actions
        declare_use_sim_time = DeclareLaunchArgument('use_sim_time',
                                            default_value='true',
                                            description='Use simulation (Gazebo) clock if true')

        declare_map = DeclareLaunchArgument('map',
                                            default_value=default_map_path,
                                            description='Navigation で使用したい map ファイルを選択してください')

        nav2_bringup = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([nav2_prefix, '/launch/bringup_launch.py']),
                        launch_arguments={'use_sim_time': use_sim_time,
                                            'map': map}.items(),
                    )
                    
        rviz = Node(package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_dir])
        '''
        print(nav2_prefix)
        print(use_sim_time)
        print(map)
        print(declare_use_sim_time)
        print(declare_map)
        print(nav2_bringup)
        '''
        ld.add_action(declare_use_sim_time)
        ld.add_action(declare_map)
        ld.add_action(nav2_bringup)
        ld.add_action(rviz)

        return ld
    except Exception as e:
        traceback.print_exc()
        raise e
