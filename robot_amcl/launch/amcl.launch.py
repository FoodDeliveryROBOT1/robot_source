import os

from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'robot_amcl'

def generate_launch_description():
  """Generates launch description for amcl node."""

  # Get path to package share directory
  nav2_dir = os.path.join(os.getenv('COLCON_PREFIX'), 'share', 'nav2')

  # Define amcl node
  amcl_node = Node(
      package='nav2_amcl',
      executable='amcl',
      name='amcl',
      parameters=[os.path.join('share', package_name, 'config/amcl.yaml')],
  )

  # Return launch description
  return LaunchDescription([amcl_node])
