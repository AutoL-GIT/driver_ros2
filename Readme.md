# ROS2 AutoL Drivers

#### Overview

- Provides a development/testing enviroment on the Linux ROS platform for verifying/handling LiDAR Data

- Real-time or stored LiDAR data can be processed and output in the form of a point cloud (utilizing the Rviz tool which provides GUI).

- If needed by the client, intermediate data can be acquired for additional processing

- Source Code: Download or Git Clone from GitHub

  

## 1. Preparation 

#### 1.1 OS Requirements 

- Ubuntu 20.04 for ROS2 Foxy
- Ubuntu 22.04 for ROS2 Humble

#### 1.2 Installation ROS2

- For ROS2 Foxy installation, please refer to: [ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

- For ROS2 Humble installation, please refer to: [ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
- ROS2 uses the Colcon build tool, please refer to: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- Desktop-Full installation is recommend.

#### 1.3 Install related library

- Install libpcap, libyaml-cpp, libpcl
  ```bash
  $ sudo apt-get update && sudo apt-get upgrade
  $ sudo apt-get install build-essential
  $ sudo apt-get install libpcap-dev
  $ sudo apt-get install libyaml-cpp-dev
  $ sudo apt-get install libpcl-all
  $ sudo apt-get install ros-${distro}-pcl-conversions ros-${distro}-pcl-ros
  ```

  

## 2. Build & Run 

#### 2.1 Clone Autol ROS2 Driver source code

```bash
$ git clone https://github.com/AutoL-GIT/driver_ros2.git
```

#### 2.2 Colcon build Autol ROS Dirver2

```bash
$ colcon build #in workspace 
$ source install/setup.bash
```

#### 2.3 Launch the Package

1. **Launch the autol_driver**

   ```bash
   $ ros2 launch autol_driver driver.py
   ```

2. **Launch the autol_pointcloud**

   ```bash
   $ ros2 launch autol_pointcloud pointcloud.py
   ```



## 3. ROS2 Interface Structure

#### 3.1 Node

| Node name       | Description                                                  |
| --------------- | :----------------------------------------------------------- |
| driver_node     | Receive the UDP Packets from the LiDAR Sensor, Package them into frame(one scene) unit, and deliver(publish). |
| pointcloud_node | Receive the Subscribes to frame unit packet data(from the driver_node), transforms data into a 3D Cartesian coordinate system (pointcloud2). |

#### 3.2 Topic

| Topic name       | Description                               |
| ---------------- | ----------------------------------------- |
| autol_frame_data | Packet data communication per frame unit. |
| autol_pointcloud | Data communication in Pointcloud2 format. |

#### 3.3 Node Graph

![image-20240226143649146](/home/autol/.config/Typora/typora-user-images/image-20240226143649146.png)

- If utilizing point cloud data is needed, use the data received through the autol_pointcloud topic.

## 4. Introduction to Launch file and Parameters

#### 4.1 Launch file

| launch file name  | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| driver.launch     | Connect to AutoL G32 LiDAR device and Publish UDP Packet format data (autol_frame_data) |
| pointcloud.launch | Publish pointcloud2 msg and auto load rviz                   |

#### 4.2 Parameter

##### 1. auto_driver parameter

| Parameter        | Detailed description                                         | Default       |
| ---------------- | ------------------------------------------------------------ | ------------- |
| manufature_id    | Set the Lidar manufacture id                                 | autol         |
| model_id         | Supported LiDAR models are listed in the README file.        | G32           |
| input_type       | Set the source of LiDAR packets<br />0 -- Unused . Never set this parameter to 0.<br />1 -- LiDAR packets come frome on-line LiDAR<br />2 -- LiDAR packets come from a PCAP | 1             |
| pcap_path        | The full path of the PCAP file. Valid if input_type = 2.     | " "           |
| packet_per_frame | Set the num of packet per frame, recommended values 180.     | 180           |
| frame_rate       | Set the frequency of point cloud publish Floating-point data type. | 25 (unit: Hz) |
| lidar_count      | Set the num of lidar                                         | 1             |
| lidar_port_1     | Set the First Lidar communication packet port.               | 5001          |
| lidar_port_2     | Set the Second Lidar communication packet port.              | 5002          |
| ⁞                | ⁞                                                            | ⁞             |
| lidar_port_6     | Set the Sixth Lidar communication packet port.               | 5006          |

```python
#Example of setting the parameters of driver launch file 
manufacture_id = 'autol'
model_id = 'G32'
input_type = 1
pcap_path = ''
packet_per_frame = 180
frame_rate = 25
lidar_count = 1
lidar_port_1 = 5001
lidar_port_2 = 5002
lidar_port_3 = 5003
lidar_port_4 = 5004
lidar_port_5 = 5005
lidar_port_6 = 5006
```

##### 2. autol_pointcloud parameter

| Parameter   | Detailed description                                         | Default |
| ----------- | ------------------------------------------------------------ | ------- |
| calibration | Set whether to use calibration(X, Y, Z, Roll, Pitch, Yaw) or not<br />0 -- unused slam offset <br />1 -- used slam offset<br /> (Calibration values can be set in the 'autol_pointcloud/params
/slam_offset.yaml' file) | False   |

```python
#Example of setting the parameters of pointcloud launch file 
calibration = False
```

## 5. Supported LiDAR List

- Manufacture id: AutoL / Model Id: G32 
- (more types are comming soon...)
