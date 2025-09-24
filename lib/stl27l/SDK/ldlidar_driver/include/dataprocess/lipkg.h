/**
 * @file lipkg.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
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

#ifndef __LIPKG_H
#define __LIPKG_H


#include <chrono>
#include <functional>
#include <mutex>

#include <string.h>

#include "ldlidar_datatype.h"
#include "tofbf.h"

namespace ldlidar {

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  LiPkg();
  ~LiPkg();
  
  // set product type (belong to enum class LDType)
  void SetProductType(LDType type_number);
  // get Lidar spin speed (Hz)
  double GetSpeed(void); 
  // get lidar spind speed (degree per second) origin
  uint16_t GetSpeedOrigin(void);
  // get time stamp of the packet
  uint16_t GetTimestamp(void);  
  // get lidar measure frequence(Hz)
  int GetLidarMeasurePointFrequence(void);
  
  void CommReadCallback(const char *byte, size_t len);

  bool GetLaserScanData(Points2D& out);

  void RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle);

  bool GetLidarPowerOnCommStatus(void);

  void EnableFilter(bool is_enable);

  LidarStatus GetLidarStatus(void);

  void ClearDataProcessStatus(void) {
    is_frame_ready_ = false;
    is_poweron_comm_normal_ = false;
    lidarstatus_ = LidarStatus::NORMAL;
    last_pkg_timestamp_ = 0;
    first_frame_ = true;
  }


 private:
  LDType product_type_;
  uint16_t timestamp_;
  double speed_;
  bool is_frame_ready_;
  bool is_poweron_comm_normal_;
  bool is_filter_;
  LidarStatus lidarstatus_;
  int measure_point_frequence_;
  std::function<uint64_t(void)> get_timestamp_;
  uint64_t last_pkg_timestamp_;
  bool first_frame_;

  LiDARFrameTypeDef pkg_;
  Points2D frame_tmp_;
  Points2D laser_scan_data_;
  std::mutex  mutex_lock1_;
  std::mutex  mutex_lock2_;
  

   // parse single packet
  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t* data, long len);  
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();  
  void SetFrameReady(void);
  void SetLaserScanData(Points2D& src);

  // Get lidar data frame ready flag  
  bool IsFrameReady(void);  
  // Lidar data frame readiness flag reset
  void ResetFrameReady(void);
};

} // namespace ldlidar

#endif  //__LIPKG_H

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/