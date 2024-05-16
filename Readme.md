# ROS2 AutoL Drivers

#### Overview

- Provides a development/testing enviroment on the Linux ROS platform for verifying/handling LiDAR Data

- Real-time or stored LiDAR data can be processed and output in the form of a point cloud (utilizing the Rviz tool which provides GUI).

- If needed by the client, intermediate data can be acquired for additional processing

- Source Code: Download or Git Clone from GitHub

- Default Sensor Network IP: 192.168.1.101

- PC Network IP: 192.168.1.xx
![driver_interface](https://github.com/AutoL-GIT/driver_ros2/assets/57899329/d585228a-c424-4bc6-8668-a391b6d84c9d)

## 1. Preparation 

#### 1.1 OS Requirements 

- Ubuntu 20.04 for ROS2 Foxy
- Ubuntu 22.04 for ROS2 Humble

#### 1.2 Installation ROS2

- ROS2 uses the Colcon build tool, please refer to: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

- For ROS2 Foxy installation, please refer to: [ROS Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- For ROS2 Humble installation, please refer to: [ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) 
- Desktop-Full installation is recommend.

#### 1.3 Install related library

- Install libpcap, libyaml-cpp, libpcl
  ```bash
  $ sudo apt-get update && sudo apt-get upgrade
  $ sudo apt-get install build-essential
  $ sudo apt-get install libpcap-dev
  $ sudo apt-get install libyaml-cpp-dev
  $ sudo apt-get install libpcl-dev
  ```


#### 1.4 Class Structure

- **class diagram**
  ![class_diagram](https://github.com/AutoL-GIT/driver_ros2/assets/57899329/e4c657cf-19e8-4188-a9b9-0920a69deab9)
  - **AutolDriver**:  AutolDriver defines the interface of a source and create ros node & topic 
  - **InputManager**: Interface class of InputSocket and InputPcap
    - **InputSocket**: Socket communication interface class
    - **InputPcap**: Pcap file interface class
  - **LidarController**: LiDAR parsing thread management class
  - **ParserManager**: LiDAR parsing management class
  - **G32Parser**: G32 parsing class

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

## 3. ROS2 Interface Structure

#### 3.1 Node

| Node name  | Description                                                  |
| ---------- | :----------------------------------------------------------- |
| pub_frame_ | Receive the UDP Packets from the LiDAR Sensor, Package them into frame(one scene) unit, and deliver(publish). |
| pub_pcd_   | Receive the Subscribes to frame unit packet data, transforms data into a 3D Cartesian coordinate system (pointcloud2). |

#### 3.2 Topic

| Topic name       | Description                               |
| ---------------- | ----------------------------------------- |
| autol_frame_data | Packet data communication per frame unit. |
| autol_pointcloud | Data communication in Pointcloud2 format. |


- If utilizing point cloud data is needed, use the data received through the autol_pointcloud topic.

## 4. Introduction to Launch file and Parameters

#### 4.1 Launch file

| launch file name | Description                                                  |
| ---------------- | ------------------------------------------------------------ |
| driver.py        | 1. Connect to AutoL G32 LiDAR device and Publish UDP Packet format data (autol_frame_data)<br />2. Publish pointcloud2 msg and auto load rviz)<br />3. Path<br />   - autol_driver/driver.py #if you modify this file, you must compile the code<br />   - autol_driver/install/driver.py #if you modify this file, you should not compile the code. but the content does not  apply source code <br /> |



#### 4.2 Parameter

##### 1. auto_driver parameter

| Parameter        | Detailed description                                         | Default       |
| ---------------- | ------------------------------------------------------------ | ------------- |
| manufature_id    | Set the Lidar manufacture id                                 | autol         |
| model_id         | Supported LiDAR models are listed in the README file.        | G32           |
| input_type       | Set the source of LiDAR packets<br />0 -- Unused . Never set this parameter to 0.<br />1 -- LiDAR packets come frome on-line LiDAR<br />2 -- LiDAR packets come from a PCAP | 1             |
| frame_rate       | Set the frequency of point cloud publish Floating-point data type. | 25 (unit: Hz) |
| lidar_count      | Set the num of lidar                                         | 1             |
| lidar_port_1     | Set the First Lidar communication packet port.               | 5001          |
| lidar_port_2     | Set the Second Lidar communication packet port.              | 5002          |
| ⁞                | ⁞                                                            | ⁞             |
| lidar_port_6     | Set the Sixth Lidar communication packet port.               | 5006          |
| pcap_path        | The full path of the PCAP file. Valid if input_type = 2.     | " "           |
| packet_per_frame | Set the num of packet per frame, recommended values 180.     | 180           |
| read_once        | Variables that determine whether to read the pcap file repeatedly or once | 0             |
| read_fast        | Adjust the pcap read speed more faster                       | 0             |
| repeat_delay     | Adjust the pcap read speed more slower                       | 0             |
| calibration      | Set whether to use calibration(X, Y, Z, Roll, Pitch, Yaw) or not<br />False -- unused slam offset <br />True -- used slam offset<br />  ※ Calibration values can be set in the 'autol_driver/params/slam_offset.yaml' file | False         |

```python
#Example of setting the parameters of driver launch file 
manufacture_id = 'autol'
model_id = 'G32'
input_type = 1
frame_rate = 25

lidar_count = 1
lidar_port_1 = 5001
lidar_port_2 = 5002
lidar_port_3 = 5003
lidar_port_4 = 5004
lidar_port_5 = 5005
lidar_port_6 = 5006

pcap_path = ''
packet_per_frame = 180
read_once = 0
read_fast = 0
repeat_delay = 0.0
calibration = True
```

## 5. Slam Offset (LiDAR Calibration)

#### 5.1 Slam Offset 

- This file is for LiDAR calibration.
- It can adjust  up to six LiDAR point cloud position for X, Y, Z, Roll, Pitch, Yaw.
- Path: autol_driver/params/slam_offset.yaml
- If you modify the file, you must compile the code

#### 5.2 Code 

```yaml
offset:
- {lidar_id: 0, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}
- {lidar_id: 1, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}
- {lidar_id: 2, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}
- {lidar_id: 3, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}
- {lidar_id: 4, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}
- {lidar_id: 5, roll: 0, pitch: 0, yaw: 0, x_offset: 0, y_offset: 0, z_offset: 0}

```



## 6. Subscribe autol_pointcloud node example

#### 6.1 To learn how to receive point cloud data topics, refer to the example_1 file

```c++
//example_1 RcvPcd.cpp
RcvPcd::RcvPcd(const rclcpp::NodeOptions &options)
    : rclcpp::Node("example_node", options)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    //Subscribe the pointcloud data
    subPointData_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("autol_pointcloud",
                                                                             qos_profile,
                                                                             std::bind(&RcvPcd::pointcloud_cb, this,
                                                                                       std::placeholders::_1));
    rclcpp::spin(this->get_node_base_interface());
}

void RcvPcd::DoSomthing()
{
    // Put your code here
}

void RcvPcd::pointcloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd)
{
    //accumlate the pointcloud data to sensor_msgs
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcd, "x"),
                                                 iter_y(*pcd, "y"),
                                                 iter_z(*pcd, "z"),
                                                 iter_intensity(*pcd, "intensity");
    int cnt = 0;
    while (iter_x != iter_x.end())
    {
        ++cnt;
        if (cnt == 10000)
        {
            RCLCPP_INFO(this->get_logger(), "%f %f %f %f", *iter_x, *iter_y, *iter_z, *iter_intensity);
        }
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
    }
}
```

## 7. Save PCD File 

#### 7.1 To learn how to save PCD , refer to the autol_pcd_saver file

- After subscribing point2cloud msg from autol_driver, save it as a pcd file.

- Launch the autol_pcd_saver
  ```bash
  $ colcon build
  $ source install setup.bash
  $ ros2 launch autol_pcd_saver autol_pcd_saver.py
  ```
#### 7.2 Launch file

- **Before starting, be sure to specify the save_path in the launch file.**

  | launch file name | Description                             |
  | ---------------- | --------------------------------------- |
  | autol_pcd_saver  | Specifies the path to the storage file. |

#### 7.3 Parameter

| Parameter | Detailed description              | Default |
| --------- | --------------------------------- | ------- |
| save_path | Set the path to the storage file. | ' '     |

```python
#Example of setting the parameters of driver launch file 
#Please type your storage path here.
save_path = ''
```

#### 7.4 Code

```C++
//Subscribe PointCloud2 Msg and Save the PCD File
void PcdSaver::SavePcdCallBack(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd)
{
    ++cnt_;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //PointCloud2 Msg -> PCD Format
    pcl::fromROSMsg(*pcd, cloud);
    std::stringstream ss;
    ss << save_path_ << cnt_ << ".pcd";

    std::string filename = ss.str();

    RCLCPP_INFO(node_.get_logger(), "pcd data : %s", filename.c_str());
    //Save PCD File
    pcl::io::savePCDFileASCII(filename, cloud);
}
```

## 8. Supported LiDAR List

- Manufacture id: AutoL / Model Id: G32 

- (more types are comming soon...)

  
