

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():

    # Define parameters for the local and global costmaps
    local_costmap_params = os.path.join(get_package_share_directory('robot_amcl'), 'config', 'example.yaml')
    global_costmap_params = os.path.join(get_package_share_directory('robot_amcl'), 'config', 'example.yaml')
    
    use_respawn = LaunchConfiguration('use_respawn')

    use_sim_time = LaunchConfiguration('use_sim_time')

    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = [        'local_costmap_node',
                                'global_costmap_node',]

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
    # Create nodes for local and global costmaps
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap_node',
        parameters=[local_costmap_params],
        output='screen'
    )

    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap_node',
        parameters=[global_costmap_params],
        output='screen'
    )
    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        declare_use_respawn_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        local_costmap_node,
        global_costmap_node,
        lifecycle_manager
    ])

if __name__ == '__main__':
    generate_launch_description()
