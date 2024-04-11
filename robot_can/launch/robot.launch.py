import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),'launch','ydlidar_launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    mapping = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot_can'),'launch','mapping.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )
    can_node = Node(
        package = 'robot_can',
        executable = 'can_node',
        output = 'screen'
    )
    odom_node = Node(
        package = 'robot_wheel_odom',
        executable = 'robot_odom',
        output = 'screen'
    )
    imu_node = Node(
        package = 'robot_imu',
        executable = 'hfi_a9_imu',
        output = 'screen'
    )
    ekf_node = Node(
        package = 'robot_localization',
        executable = 'ekf_v2',
        output = 'screen'
    )


    ld = LaunchDescription()
    ld.add_action(lidar)
    ld.add_action(can_node)
    ld.add_action(odom_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)
  
    ld.add_action(use_sim_time_arg)

    

    return ld