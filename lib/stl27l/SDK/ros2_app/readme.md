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

  - 修改`ros2_app/src/ldlidar/launch/ (产品型号).launch.py`文件中的port_name值，以`ld06.launch.py`和`/dev/ttyUSB0`为例，如下所示.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
- Serial port communication mode or network port communication mode configuration
  --enable serial communication mode:
  {'enable_serial_or_network_communication': True}
  --enable network communication mode:
  {'enable_serial_or_network_communication': False}
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'enable_serial_or_network_communication': True},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'server_ip': '192.168.1.200'},
        {'server_port': '2000'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0},
        {'measure_point_freq': 4500}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld06',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
## 2. 编译方法

使用colcon编译，在readme文件所在目录下执行如下指令.

```bash
colcon build
```
## 3. 运行方法

```bash
source install/local_setup.bash
```
- 产品型号为 LDROBOT LiDAR LD06

  ``` bash
  ros2 launch ldlidar ld06.launch.py  # start node 

  ros2 launch ldlidar viewer_ld06.launch.py # start node and show data on the Rviz2
  ```

- 产品型号为 LDROBOT LiDAR LD19

  ``` bash
  ros2 launch ldlidar ld19.launch.py  # start node 

  ros2 launch ldlidar viewer_ld19.launch.py # start node and show data on the Rviz2
  ```

- 产品型号为 LDROBOT LiDAR STL06P

  ``` bash
  ros2 launch ldlidar stl06p.launch.py  # start node 

  ros2 launch ldlidar viewer_stl06p.launch.py # start node and show data on the Rviz2
  ```

- 产品型号为 LDROBOT LiDAR STL27L

  ``` bash
  ros2 launch ldlidar stl27l.launch.py  # start node 

  ros2 launch ldlidar viewer_stl27l.launch.py # start node and show data on the Rviz2
  ```

- 产品型号为 LDROBOT LiDAR STL26

  ``` bash
  ros2 launch ldlidar stl26.launch.py  # start node 

  ros2 launch ldlidar viewer_stl26.launch.py # start node and show data on the Rviz2
  ```


##   4. 测试

> 代码支持ubuntu20.04 ROS2 foxy版本及以上测试，使用rviz2可视化。
- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz2工具打开readme文件所在目录的rviz2文件夹下面的ldlidar.rviz文件
```bash
rviz2
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
  -  Modify port_name value in the `ros2_app/src/ldlidar/launch/`(product name).launch  files,

   > for example `ld06.launch.py` and `/dev/ttyUSB0`.

``` py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
- Serial port communication mode or network port communication mode configuration
  --enable serial communication mode:
  {'enable_serial_or_network_communication': True}
  --enable network communication mode:
  {'enable_serial_or_network_communication': False}
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'enable_serial_or_network_communication': True},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'server_ip': '192.168.1.200'},
        {'server_port': '2000'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0},
        {'measure_point_freq': 4500}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld06',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
## step 2: build

Run the following command in the directory where the readme file resides.

```bash
colcon build
```
## step 3: run

```bash
source install/local_setup.bash
```
- The product is LDROBOT LiDAR LD06
  ``` bash
  ros2 launch ldlidar ld06.launch.py  # start node 

  ros2 launch ldlidar viewer_ld06.launch.py # start node and show data on the Rviz2
  ```

- The product is LDROBOT LiDAR LD19
  ``` bash
  ros2 launch ldlidar ld19.launch.py  # start node 

  ros2 launch ldlidar viewer_ld19.launch.py # start node and show data on the Rviz2
  ```

- The product is LDROBOT LiDAR STL06P
  ``` bash
  ros2 launch ldlidar stl06p.launch.py  # start node 

  ros2 launch ldlidar viewer_stl06p.launch.py # start node and show data on the Rviz2
  ```

- The product is LDROBOT LiDAR STL27L
  ``` bash
  ros2 launch ldlidar stl27l.launch.py  # start node 

  ros2 launch ldlidar viewer_stl27l.launch.py # start node and show data on the Rviz2
  ```

- The product is LDROBOT LiDAR STL26
  ``` bash
  ros2 launch ldlidar stl26.launch.py  # start node 

  ros2 launch ldlidar viewer_stl26.launch.py # start node and show data on the Rviz2
  ```

## step 3: test

> The code supports ubuntu 20.04 ros2 foxy version and above, using rviz2 visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz2 tool ,open the `ldlidar.rviz` file below the rviz2 folder of the readme file directory
```bash
rviz2
```

[ Back to the introduction ](../README.md)

