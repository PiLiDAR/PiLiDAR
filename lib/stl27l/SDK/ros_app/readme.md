- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品.

## 1. 系统设置
- 将雷达连接到你的系统主板，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
sudo chmod 777 /dev/ttyUSB0
```

  - 修改`ros_app/src/ldlidar/launch/ <产品型号>.launch`文件中的port_name值，以`/dev/ttyUSB0` , `ld06.launch`为例，如下所示.

```xml
<launch>
<!-- ldldiar message publisher node -->
 <node name="LD06" pkg="ldlidar" type="ldlidar" output="screen" >
  <param name="product_name" value="LDLiDAR_LD06"/>
  <param name="topic_name" value="scan"/>
  <param name="frame_id" value="base_laser"/>
  <!--Serial port communication mode or network port communication mode configuration-->
  <!--enable serial communication mode:-->
  <!-- <param name="enable_serial_or_network_communication" value="true"/>  -->
  <!--enable network communication mode:-->
  <!-- <param name="enable_serial_or_network_communication" value="false"/> -->
  <param name="enable_serial_or_network_communication" value="true"/>
  <param name="port_name" value ="/dev/ttyUSB0"/>
  <param name="port_baudrate" value ="230400"/>
  <param name="server_ip" value="192.168.1.200"/>
  <param name="server_port" value="2000"/>
  <!-- Set laser scan directon: -->
  <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
  <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
  <param name="laser_scan_dir" type="bool" value="true"/>
  <!-- Angle crop setting, Mask data within the set angle range -->
  <!--    1. Enable angle crop fuction: -->
  <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
  <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
  <param name="enable_angle_crop_func" type="bool" value="false"/>
  <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
  <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
  <param name="angle_crop_min" type="double" value="135.0"/>
  <param name="angle_crop_max" type="double" value="225.0"/>
  <param name="measure_point_freq" type="int" value="4500"/>
 </node>
 <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
 <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
 <node name="base_to_laser_ld06" pkg="tf" type="static_transform_publisher"  args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"/>
</launch>
```
## 2. 编译方法

使用catkin编译，在readme文件所在目录下执行如下指令.

```bash
catkin_make
```
## 3. 运行方法

```bash
source devel/setup.bash
```
- 产品型号为 LDROBOT LiDAR LD06

  - 启动ld06 lidar node:
  ``` bash
  roslaunch ldlidar ld06.launch
  ```
  - 启动ld06 lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_ld06_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_ld06_noetic.launch
  ```

- 产品型号为 LDROBOT LiDAR LD19

  - 启动ld19 lidar node:
  ``` bash
  roslaunch ldlidar ld19.launch
  ```
  - 启动ld19 lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_ld19_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_ld19_noetic.launch
  ```

- 产品型号为 LDROBOT LiDAR STL06P

  - 启动stl06p lidar node:
  ``` bash
  roslaunch ldlidar stl06p.launch
  ```
  - 启动stl06p lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl06p_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl06p_noetic.launch
  ```
- 产品型号为 LDROBOT LiDAR STL27L

  - 启动stl27l lidar node:
  ``` bash
  roslaunch ldlidar stl27l.launch
  ```
  - 启动stl27l lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl27l_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl27l_noetic.launch
  ```

- 产品型号为 LDROBOT LiDAR STL26

  - 启动stl26 lidar node:
  ``` bash
  roslaunch ldlidar stl26.launch
  ```
  - 启动stl26 lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl26_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl26_noetic.launch
  ```
##   4. 测试

> 代码支持ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic版本下测试，使用rviz可视化。

- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz工具打开readme文件所在目录的rviz文件夹下面的ldlidar.rviz文件
```bash
rviz
```

[回到仓库简介](../README.md)

# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. 

## step 1: system setup
- Set the permission of serial port device mounted by LiDAR in the system(example:device name is /dev/ttyUSB0)
    - The actual use of the radar is based on the actual mounted on your system, you can use the `ls -l /dev` command to view. 

``` bash
sudo chmod 777 /dev/ttyUSB0
```
  -  Modify port_name value in the `ros_sdk/src/ldlidar/launch/` `product type`.launch  files,

   > for example `/dev/ttyUSB0`, `ld06.launch`.

``` xml
<launch>
<!-- ldldiar message publisher node -->
 <node name="LD06" pkg="ldlidar" type="ldlidar" output="screen" >
  <param name="product_name" value="LDLiDAR_LD06"/>
  <param name="topic_name" value="scan"/>
  <param name="frame_id" value="base_laser"/>
  <!--Serial port communication mode or network port communication mode configuration-->
  <!--enable serial communication mode:-->
  <!-- <param name="enable_serial_or_network_communication" value="true"/>  -->
  <!--enable network communication mode:-->
  <!-- <param name="enable_serial_or_network_communication" value="false"/> -->
  <param name="enable_serial_or_network_communication" value="true"/>
  <param name="port_name" value ="/dev/ttyUSB0"/>
  <param name="port_baudrate" value ="230400"/>
  <param name="server_ip" value="192.168.1.200"/>
  <param name="server_port" value="2000"/>
  <!-- Set laser scan directon: -->
  <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
  <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
  <param name="laser_scan_dir" type="bool" value="true"/>
  <!-- Angle crop setting, Mask data within the set angle range -->
  <!--    1. Enable angle crop fuction: -->
  <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
  <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
  <param name="enable_angle_crop_func" type="bool" value="false"/>
  <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
  <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
  <param name="angle_crop_min" type="double" value="135.0"/>
  <param name="angle_crop_max" type="double" value="225.0"/>
  <param name="measure_point_freq" type="int" value="4500"/>
 </node>
 <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
 <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
 <node name="base_to_laser_ld06" pkg="tf" type="static_transform_publisher"  args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50"/>
</launch>
```
## step 2: build

Run the following command in the directory where the readme file resides.

```bash
catkin_make
```
## step 3: run

```bash
source devel/setup.bash
```
- The product is LDROBOT LiDAR LD06

  - Start ld06 lidar node:
  ``` bash
  roslaunch ldlidar ld06.launch
  ```
  - Start ld06 lidar node and show laser data on the Rviz tool:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_ld06_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_ld06_noetic.launch
  ```

- The product is LDROBOT LiDAR LD19

  - Start ld19 lidar node:
  ``` bash
  roslaunch ldlidar ld19.launch
  ```
  - Start ld19 lidar node and show laser data on the Rviz tool:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_ld19_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_ld19_noetic.launch
  ```

- The product is LDROBOT LiDAR STL06P

  - Start stl06p lidar node:
  ``` bash
  roslaunch ldlidar stl06p.launch
  ```
  - Start stl06p lidar node and show laser data on the Rviz tool:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl06p_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl06p_noetic.launch
  ```

- The product is LDROBOT LiDAR STL27L

  - Start stl27l lidar node:
  ``` bash
  roslaunch ldlidar stl27l.launch
  ```
  - Start stl27l lidar node and show laser data on the Rviz tool:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl27l_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl27l_noetic.launch
  ```

- The product is LDROBOT LiDAR STL26

  - Start stl26 lidar node:
  ``` bash
  roslaunch ldlidar stl26.launch
  ```
  - Start stl26 lidar node and show laser data on the Rviz tool:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar viewer_stl26_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar viewer_stl26_noetic.launch
  ```
## step 3: test

> The code was tested under ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic, using rviz visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz tool,open the `ldlidar.rviz` file below the rviz folder of the readme file directory
```bash
rviz
```

[ Back to the introduction ](../README.md)

