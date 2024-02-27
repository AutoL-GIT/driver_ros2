# -*- mode: Python -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch


################### user configure parameters for ros2 start ###################
manufacture_id = 'autol'
model_id = 'G32'
input_type = 1
pcap_path = ''
packet_per_frame = 180
frame_rate = 25
lidar_count = 2
lidar_port_1 = 5001
lidar_port_2 = 5002
lidar_port_3 = 5003
lidar_port_4 = 5004
lidar_port_5 = 5005
lidar_port_6 = 5006
 
autol_node_parameters = [
    {"manufacture_id": manufacture_id},
    {"model_id": model_id},
    {"input_type" : input_type},
    {"pcap_path": pcap_path},
    {"packet_per_frame": packet_per_frame},
    {"frame_rate": frame_rate},
    {"lidar_count": lidar_count},
    {"lidar_port_1": lidar_port_1},
    {"lidar_port_2": lidar_port_2},
    {"lidar_port_3": lidar_port_3},
    {"lidar_port_4": lidar_port_4},
    {"lidar_port_5": lidar_port_5},
    {"lidar_port_6": lidar_port_6},
]

def generate_launch_description():
  autol_driver = Node(
    package='autol_driver',
    executable='autol_driver_node',
    name='autol_lidar_publisher',
    output='screen',
    parameters=autol_node_parameters,
  )
  return LaunchDescription([
    autol_driver,
  ])
    

    
