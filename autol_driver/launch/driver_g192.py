# -*- mode: Python -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
manufacture_id = 'autol'
model_id = 'G192'
data_format_version = 1
input_type = 1

lidar_count = 1
lidar_port_1 = 33233
lidar_port_2 = 33234
lidar_port_3 = 33235
lidar_port_4 = 33236
lidar_port_5 = 33237
lidar_port_6 = 33238

pcap_path = '/home/autol/data/2025-10-15-20-48-38_AutoLG192_Point_Data.pcap'
frame_rate = 10
packet_per_frame = 0
read_once = 0
read_fast = 1
calibration = True
horizon_cal_file_path = '/home/autol/data/cal_files_g192/g192_horizontal_angle_calibration.csv'
vertical_cal_file_path = '/home/autol/data/cal_files_g192/g192_vertical_angle_calibration.csv'

rviz_config=get_package_share_directory('autol_driver')+'/rviz/pointcloud2_config.rviz'
 
autol_node_parameters = [
  #Sensor Parameter
    {"manufacture_id": manufacture_id},
    {"model_id": model_id},
    {"data_format_version": data_format_version},
    {"input_type" : input_type},
    
  #Socket Parameter
    {"lidar_count": lidar_count},
    {"lidar_port_1": lidar_port_1},
    {"lidar_port_2": lidar_port_2},
    {"lidar_port_3": lidar_port_3},
    {"lidar_port_4": lidar_port_4},
    {"lidar_port_5": lidar_port_5},
    {"lidar_port_6": lidar_port_6},
  #Pcap Parameter
    {"pcap_path": pcap_path},
    {"frame_rate": frame_rate},
    {"packet_per_frame": packet_per_frame},
    {"read_once": read_once},
    {"read_fast": read_fast},
  #calibration
    {"calibration" : calibration},    
    {"horizon_cal_file_path" : horizon_cal_file_path},
    {"vertical_cal_file_path" : vertical_cal_file_path},  
]

def generate_launch_description():
  autol_driver = Node(
    package='autol_driver',
    executable='autol_driver_node',
    name='autol_lidar_publisher',
    output='screen',
    parameters=autol_node_parameters,
  )
  

  autol_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    output='screen',
    arguments=['-d',rviz_config]
  )
  return LaunchDescription([
    autol_driver, autol_rviz,
  ])
    

    
