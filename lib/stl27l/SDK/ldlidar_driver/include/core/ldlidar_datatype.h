/**
 * @file pointdata.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  lidar point data structure
 *         This code is only applicable to LDROBOT products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
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
#ifndef _LDLIDAR_POINT_DATA_H_
#define _LDLIDAR_POINT_DATA_H_

#include <stdint.h>

#include <iostream>
#include <vector>

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLED(angle) ((angle)*180000 / 3141.59)

namespace ldlidar {

enum class LDType {
  NO_VERSION,
  LD_06,
  LD_19,
  STL_06P,
  STL_26,
  STL_27L,
};

enum class LidarStatus {
  NORMAL,     //  雷达正常,可获取点云数据
  ERROR,      //  表明雷达出现异常错误，可获取雷达反馈的错误码了解具体错误，具体错误由错误码对应二进制值的对应位决定
  DATA_TIME_OUT, //  雷达点云数据包发布超时
  DATA_WAIT,  //  雷达点云数据包发布等待
  STOP,       //  未启动Driver 
};

struct PointData {
  // Polar coordinate representation
  float angle;         // Angle ranges from 0 to 359 degrees
  uint16_t distance;   // Distance is measured in millimeters
  uint8_t intensity;  // Intensity is 0 to 255
  //! System time when first range was measured in nanoseconds
  uint64_t stamp;
  // Cartesian coordinate representation
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t intensity, uint64_t stamp = 0, double x = 0,
            double y = 0) {
    this->angle = angle;
    this->distance = distance;
    this->intensity = intensity;
    this->stamp = stamp;
    this->x = x;
    this->y = y;
  }
  PointData() {}
};

typedef std::vector<PointData> Points2D;

struct LaserScan {
  //! System time when first range was measured in nanoseconds
  uint64_t stamp;
  //! Array of laser point
  Points2D points;
  
  LaserScan &operator=(const LaserScan &data) {
    this->stamp = data.stamp;
    this->points = data.points;
    return *this;
  }
};

} // namespace ldlidar

#endif  // _POINT_DATA_H_

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/