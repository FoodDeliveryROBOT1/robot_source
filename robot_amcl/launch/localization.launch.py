import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('robot_amcl'), 'config', 'nav2_param.yaml')

    # rviz_config_dir = os.path.join(get_package_share_directory('robot_amcl'), 'config', 'rviz_config.rviz')

    map_file = os.path.join(get_package_share_directory('robot_amcl'), 'maps', 'map_11_april.yaml')

    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    
    use_respawn = LaunchConfiguration('use_respawn')

    use_sim_time = LaunchConfiguration('use_sim_time')

    autostart = LaunchConfiguration('autostart')

    use_respawn = LaunchConfiguration('use_respawn')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    

    lifecycle_nodes = ['map_server', 'amcl']


    return LaunchDescription([
        declare_use_respawn_cmd,
        declare_autostart_cmd,
        declare_use_sim_time_cmd,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            respawn=use_respawn,
            respawn_delay=2.0,
            output='screen',
            parameters=[nav2_yaml, {'yaml_filename':map_file} ],
            remappings=remappings),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            respawn=use_respawn,
            respawn_delay=2.0,
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings
            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

        # Node(
        #     package='rviz2',
        #     node_executable='rviz2',
        #     node_name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': True}],
        #     output='screen')
        ])


## ros2 launch nav2_bringup localization_launch.py map:=./use_map.yaml use_sim_time:=false