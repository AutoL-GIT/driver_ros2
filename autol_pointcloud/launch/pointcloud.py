# -*- mode: Python -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

calibration = False

def generate_launch_description():
    autol_pointcloud = Node(
        package='autol_pointcloud',
        executable='autol_pointcloud_node',
        name='autol_pointcloud_publisher',
        parameters=[{'calibration': calibration}]  # Set your default parameter value
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '$(find autol_pointcloud)/rviz/pointcloud2_config.rviz']
    ) 
    return LaunchDescription([
       autol_pointcloud, rviz,
    ])
