# -*- mode: Python -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
#Example of setting the parameters of driver launch file 
#Please type your storage path here.
save_path = ''
 
autol_node_parameters = [
    {"save_path": save_path},
]

def generate_launch_description():
  autol_pcd_saver = Node(
    package='autol_pcd_saver',
    executable='autol_pcd_saver_node',
    name='autol_pcd_publisher',
    output='screen',
    parameters=autol_node_parameters,
  )
  
  return LaunchDescription([
    autol_pcd_saver, 
  ])
    

    
