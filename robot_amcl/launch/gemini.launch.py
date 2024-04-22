from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  # Set robot radius directly
  robot_radius = 0.22  # Replace with your robot radius

  # Local Costmap Node
  local_costmap = Node(
      package='nav2_costmap_2d',
      executable='costmap_2d_node',
      name='local_costmap',
      parameters=[
          ('use_plugin', True),
          ('global_frame', 'odom'),
          ('robot_base_frame', 'base_link'),
          ('update_frequency', 5.0),
          ('publish_frequency', 2.0),
          ('resolution', 0.05),
          ('width', 3.0),
          ('height', 3.0),
          ('rolling_window', True),
          ('always_send_full_costmap', True),
          ('robot_radius', robot_radius),
      ],
      remappings=[
          ('voxel_layer.yaml', '$(find nav2_costmap_2d)/config/voxel_layer.yaml'),
          ('inflation_layer.yaml', '$(find nav2_costmap_2d)/config/inflation_layer.yaml'),
      ]
  )

  # Global Costmap Node (unchanged)
  global_costmap = Node(
      package='nav2_costmap_2d',
      executable='costmap_2d_node',
      name='global_costmap',
      parameters=[
          ('use_plugin', True),
          ('global_frame', 'map'),
          ('robot_base_frame', 'base_link'),
          ('update_frequency', 1.0),
          ('publish_frequency', 1.0),
          ('resolution', 0.05),
          ('track_unknown_space', True),
          ('always_send_full_costmap', True),
      ],
      remappings=[
          ('static_layer.yaml', '$(find nav2_costmap_2d)/config/static_layer.yaml'),
          ('obstacle_layer.yaml', '$(find nav2_costmap_2d)/config/obstacle_layer.yaml'),
          ('inflation_layer.yaml', '$(find nav2_costmap_2d)/config/inflation_layer.yaml'),
      ]
  )

  return LaunchDescription([local_costmap, global_costmap])

# Launch the costmap nodes
ld = generate_launch_description()
