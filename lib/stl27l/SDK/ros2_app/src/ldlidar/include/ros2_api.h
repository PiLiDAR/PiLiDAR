/**
 * @file ros_api.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  ros lib interface
 *         This code is only applicable to LDROBOT LiDAR products 
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __ROS_API_H__
#define __ROS_API_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

struct LaserScanSetting {
  std::string frame_id;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
  int measure_point_freq;
};

#endif //__ROS_API_H__

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/