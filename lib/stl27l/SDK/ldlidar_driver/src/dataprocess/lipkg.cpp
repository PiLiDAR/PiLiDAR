/**
 * @file lipkg.cpp
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

#include "lipkg.h"

namespace ldlidar {

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg() 
  : product_type_(LDType::NO_VERSION),
    timestamp_(0),
    speed_(0),
    is_frame_ready_(false),
    is_poweron_comm_normal_(false),
    is_filter_(true),
    lidarstatus_(LidarStatus::NORMAL),
    measure_point_frequence_(4500),
    get_timestamp_(nullptr),
    last_pkg_timestamp_(0),
    first_frame_(true) {

}

LiPkg::~LiPkg() {

}

void LiPkg::SetProductType(LDType type_number) {
  product_type_ = type_number;
  switch (type_number) {
    case LDType::LD_06:
    case LDType::LD_19:
      measure_point_frequence_ = 4500;
      break;
    case LDType::STL_06P:
    case LDType::STL_26:
      measure_point_frequence_ = 5000;
      break;
    case LDType::STL_27L:
      measure_point_frequence_ = 21600;
      break;
    default : 
      measure_point_frequence_ = 4500;
      break;
  }
}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == PKG_HEADER) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if (byte == PKG_VER_LEN) {
        tmp[count++] = byte;
        state = DATA;
      } else {
        state = HEADER;
        count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&pkg_, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&pkg_, pkg_count - 1);
        state = HEADER;
        count = 0;
        if (crc == pkg_.crc8) {
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false; 
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisOne(data[i])) {
      is_poweron_comm_normal_ = true;
      // parse a package is success
      double diff = (pkg_.end_angle / 100 - pkg_.start_angle / 100 + 360) % 360;
      if (diff <= ((double)pkg_.speed * POINT_PER_PACK / measure_point_frequence_ * 1.5)) {
        
        if (0 == last_pkg_timestamp_) {
          last_pkg_timestamp_ = get_timestamp_();
          continue;
        }
        uint64_t current_pack_stamp = get_timestamp_();
        int pkg_point_number = POINT_PER_PACK;
        double pack_stamp_point_step =  
          static_cast<double>(current_pack_stamp - last_pkg_timestamp_) / static_cast<double>(pkg_point_number - 1);

        speed_ = pkg_.speed; // Degrees per second
        timestamp_ = pkg_.timestamp; // In milliseconds
        uint32_t diff = ((uint32_t)pkg_.end_angle + 36000 - (uint32_t)pkg_.start_angle) % 36000;
        float step = diff / (POINT_PER_PACK - 1) / 100.0;
        float start = (double)pkg_.start_angle / 100.0;
        PointData data;
        for (int i = 0; i < POINT_PER_PACK; i++) {
          data.distance = pkg_.point[i].distance;
          data.angle = start + i * step;
          if (data.angle >= 360.0) {
            data.angle -= 360.0;
          }
          data.intensity = pkg_.point[i].intensity;
          data.stamp = static_cast<uint64_t>(last_pkg_timestamp_ + (pack_stamp_point_step * i));
          frame_tmp_.push_back(PointData(data.angle, data.distance, data.intensity, data.stamp));
        }

        last_pkg_timestamp_ = current_pack_stamp; //// update last pkg timestamp
      }
    }
  }

  return true;
}

bool LiPkg::AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  for (auto n : frame_tmp_) {
    // wait for enough data, need enough data to show a circle
    // enough data has been obtained
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      if ((count * GetSpeed()) > (measure_point_frequence_ * 1.4)) {
        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        }
        return false;
      }

      data.insert(data.begin(), frame_tmp_.begin(), frame_tmp_.begin() + count);

      if (is_filter_) {
        Tofbf tofbfLd(speed_, product_type_);
        tmp = tofbfLd.Filter(data);
      } else {
        tmp = data;
      }
    
      std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) { return a.stamp < b.stamp; });

      if (tmp.size() > 0) {
        if (first_frame_) {
          first_frame_ = false;
        } else {
          SetLaserScanData(tmp);
          SetFrameReady();
        }

        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        }
        return true;
      }
    }
    count++;

    if ((count * GetSpeed()) > (measure_point_frequence_ * 2)) {
      if (count >= (int)frame_tmp_.size()) {
        frame_tmp_.clear();
      } else {
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
      }
      return false;
    }

    last_angle = n.angle;
  }

  return false;
}

double LiPkg::GetSpeed(void) {
  return (speed_ / 360.0);  // unit is hz
}

uint16_t LiPkg::GetSpeedOrigin(void) {
  return speed_;
}

uint16_t LiPkg::GetTimestamp(void) {
  return timestamp_; 
}

int LiPkg::GetLidarMeasurePointFrequence(void) {
  return measure_point_frequence_;
}

bool LiPkg::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return is_frame_ready_; 
}  

void LiPkg::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = false;
}

void LiPkg::SetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = true;
}

bool LiPkg::GetLaserScanData(Points2D& out) {
  if (IsFrameReady()) {
    ResetFrameReady();
    {
      std::lock_guard<std::mutex> lg(mutex_lock2_);
      out = laser_scan_data_;
    }
    return true;
  } else {
    return false;
  }
}

void LiPkg::SetLaserScanData(Points2D& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  laser_scan_data_ = src;
}

void LiPkg::RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle) {
  get_timestamp_ = timestamp_handle;
}

bool LiPkg::GetLidarPowerOnCommStatus(void) {
  if (is_poweron_comm_normal_) {
    is_poweron_comm_normal_ = false;
    return true;
  } else {
    return false;
  }
}

void LiPkg::EnableFilter(bool is_enable) {
  is_filter_ = is_enable;
}

LidarStatus LiPkg::GetLidarStatus(void) {
  return lidarstatus_;
}

void LiPkg::CommReadCallback(const char *byte, size_t len) {
  if (this->Parse((uint8_t *)byte, len)) {
    this->AssemblePacket();
  }
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/