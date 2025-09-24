/**
 * @file tofbf.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR near-range filtering algorithm
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

#include "tofbf.h"

namespace ldlidar {

/**
 * @brief Construct a new Tofbf:: Tofbf object
 * @param [in]
 *  @param speed  current lidar speed
 */
Tofbf::Tofbf(int speed, LDType type) { 
  curr_speed_ = speed; // ldliar spin speed, unit is Degrees per second
  switch (type)
  {
    case LDType::LD_06:
    case LDType::LD_19:
      intensity_low_ = 15;
      intensity_single_ = 220;
      scan_frequency_ = 4500;
      filter_type_ = FilterType::NEAR_FILTER;
      break;
    case LDType::STL_06P:
    case LDType::STL_26:
    case LDType::STL_27L:
      filter_type_ = FilterType::SMALL_OBSTACLE_FILTER;
      break;
    default:
      std::cout << "[ldrobot] tofbf input ldlidar type error!" << std::endl;
      filter_type_ = FilterType::NO_FILTER;
      break;
  }
}

Tofbf::~Tofbf() {}

/**
 * @brief The tof lidar filter
 * @param [in]
 * 	@param tmp lidar point data
 * @return std::vector<PointData>
 */
std::vector<PointData> Tofbf::Filter(
  const std::vector<PointData> &tmp) const {
  std::vector<PointData> normal;

  switch(filter_type_){
    case FilterType::NEAR_FILTER:
      normal = NearFilter(tmp);
      break;

    case FilterType::NOISE_FILTER:
      normal = NoiseFilter(tmp);
      break;

    default:
      normal = tmp;
      break;
  }

  return normal;
}

std::vector<PointData> Tofbf::NearFilter(
    const std::vector<PointData> &tmp) const {
  std::vector<PointData> normal, pending, item;
  std::vector<std::vector<PointData>> group;

  // Remove points within 5m
  for (auto n : tmp) {
    if (n.distance < 5000) {
      pending.push_back(n);
    } else {
      normal.push_back(n);
    }
  }

  if (tmp.empty()) return normal;

  double angle_delta_up_limit = curr_speed_ / scan_frequency_ * 2;

  // sort
  std::sort(pending.begin(), pending.end(),
            [](PointData a, PointData b) { return a.angle < b.angle; });

  PointData last(-10, 0, 0);
  // group
  for (auto n : pending) {
    if (fabs(n.angle - last.angle) > angle_delta_up_limit ||
        fabs(n.distance - last.distance) > last.distance * 0.03) {
      if (item.empty() == false) {
        group.push_back(item);
        item.clear();
      }
    }
    item.push_back(n);
    last = n;
  }
  // push back last item
  if (item.empty() == false) group.push_back(item);

  if (group.empty()) return normal;

  // Connection 0 degree and 359 degree
  auto first_item = group.front().front();
  auto last_item = group.back().back();
  if (fabs(first_item.angle + 360.f - last_item.angle) < angle_delta_up_limit &&
      fabs(first_item.distance - last_item.distance) < last.distance * 0.03) {
    group.front().insert(group.front().begin(), group.back().begin(), group.back().end());
    group.erase(group.end() - 1);
  }
  // selection
  for (auto n : group) {
    if (n.size() == 0) continue;
    // No filtering if there are many points
    if (n.size() > 15) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }

    // Filter out those with few points
    if (n.size() < 3) {
      int c = 0;
      for (auto m : n) {
        c += m.intensity;
      }
      c /= n.size();
      if (c < intensity_single_) {
        for (auto& point: n) {
          point.distance = 0;
          point.intensity = 0;
        }
        normal.insert(normal.end(), n.begin(), n.end());
        continue;
      }
    }

    // Calculate the mean value of distance and intensity
    double intensity_avg = 0;
    double dis_avg = 0;
    for (auto m : n) {
      intensity_avg += m.intensity;
      dis_avg += m.distance;
    }
    intensity_avg /= n.size();
    dis_avg /= n.size();

    // High intensity, no filtering
    if (intensity_avg > intensity_low_) {
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    } else {
      for (auto& point : n) {
        point.distance = 0;
        point.intensity = 0;
      }
      normal.insert(normal.end(), n.begin(), n.end());
      continue;
    }
  }

  return normal;
}

std::vector<PointData> Tofbf::NoiseFilter(
    const std::vector<PointData> &tmp) const {
  std::vector<PointData> normal;
  PointData  last_data, next_data;

  if (tmp.empty()) return normal;

  // Traversing the point data
  int count = 0;
  for (auto n : tmp) {
    if(count == 0) {
      last_data = tmp[tmp.size() - 1];
    } else {
      last_data = tmp[count - 1];
    }

    if(count == (int)(tmp.size() - 1)) {
      next_data = tmp[0];
    } else {
      next_data =tmp[count + 1];
    }
    count++;

    // Remove points with the opposite trend within 500mm
    if(n.distance < 500) {
      if((n.distance + 10 < last_data.distance && n.distance + 10 < next_data.distance)
        || (n.distance > last_data.distance + 10 && n.distance > next_data.distance + 10)) {
        if(n.intensity < 60) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      } else if((n.distance + 7 < last_data.distance && n.distance + 7 < next_data.distance)
              || (n.distance > last_data.distance + 7 && n.distance > next_data.distance + 7)) {
        if(n.intensity < 45) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      } else if((n.distance + 5 < last_data.distance && n.distance + 5 < next_data.distance)
              || (n.distance > last_data.distance + 5 && n.distance > next_data.distance + 5)) {
        if(n.intensity < 30) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      }
    }

    // Remove points with very low intensity within 5m
    if (n.distance < 5000) {
      if(n.distance < 200) {
        if(n.intensity < 25) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      } else {
        if(n.intensity < 10) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      }

      if((n.distance + 30 < last_data.distance || n.distance > last_data.distance + 30)
          &&(n.distance + 30 < next_data.distance || n.distance > next_data.distance + 30)) {
        if((n.distance < 2000 && n.intensity < 30) || n.intensity < 20) {
          n.intensity = 0;
          n.distance = 0;
          normal.push_back(n);
          continue;
        }
      }
    }
    normal.push_back(n);
  }
  return normal;
}

std::vector<PointData> Tofbf::SmallObstacleFilter(
    const std::vector<PointData> &tmp) const {   

    std::vector<PointData> normal;

    PointData left_point;
    PointData left2_point;
    PointData right_point;
    PointData right2_point;

    uint16_t left_diff_distance;
    uint16_t left2_diff_distance;
    uint16_t right_diff_distance;
    uint16_t right2_diff_distance;

    uint16_t point_id=0;

    for (auto n : tmp) {
        
        left_point   = tmp[(point_id + tmp.size() - 1) % tmp.size()];
        right_point  = tmp[(point_id + tmp.size() + 1) % tmp.size()];
        left2_point  = tmp[(point_id + tmp.size() - 2) % tmp.size()];
        right2_point = tmp[(point_id + tmp.size() + 2) % tmp.size()];

        left_diff_distance   = abs((int)n.distance - (int)left_point.distance );
        right_diff_distance  = abs((int)n.distance - (int)right_point.distance);
        left2_diff_distance  = abs((int)n.distance - (int)left2_point.distance);
        right2_diff_distance = abs((int)n.distance - (int)right2_point.distance);
        
        point_id++;
        
        if (left_diff_distance > 300 
            && ((right_diff_distance > 4 && right_diff_distance < 40) ||(right2_diff_distance > 4 && right2_diff_distance < 40))
            && n.distance < 1500) {
            n.intensity = 0;
            n.distance = 0;
            normal.push_back(n);
            continue;

        }

        if (right_diff_distance > 300 
            && ((left_diff_distance > 4 && left_diff_distance < 40) || (left2_diff_distance > 4 && left2_diff_distance < 40)) 
            && n.distance < 1500) {
            n.intensity = 0;
            n.distance = 0;
            normal.push_back(n);
            continue;
        }

        if (left2_diff_distance > 300 
            && right_diff_distance > 4 && right_diff_distance < 40 
            && n.distance < 1500) {
            n.intensity = 0;
            n.distance = 0;
            normal.push_back(n);
            continue;
        }

        if (right2_diff_distance > 300
            && left_diff_distance > 4 && left_diff_distance < 40 
            && n.distance < 1500) {
            n.intensity = 0;
            n.distance = 0;
            normal.push_back(n);
            continue;
        }

        normal.push_back(n);
    }
    return normal;

}

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/