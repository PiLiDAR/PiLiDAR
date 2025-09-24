/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
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

#include "ros_api.h"
#include "ldlidar_driver.h"

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
  LaserScanSetting& setting, ros::Publisher& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  int serial_port_baudrate;
  std::string server_ip;
  std::string server_port;
  bool enable_serial_or_network_communication;
  LaserScanSetting setting;
  ldlidar::LDType type_name;
	
  nh_private.getParam("product_name", product_name);
	nh_private.getParam("topic_name", topic_name);
  nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
  nh_private.param("enable_serial_or_network_communication", enable_serial_or_network_communication, bool(true));
	nh_private.getParam("port_name", port_name);
  nh_private.param("port_baudrate", serial_port_baudrate, int(230400));
  nh_private.param("server_ip", server_ip, std::string("192.168.1.200"));
  nh_private.param("server_port", server_port, std::string("2000"));
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));
  nh_private.param("measure_point_freq", setting.measure_point_freq, int(4500));

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  ROS_INFO("LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  ROS_INFO("ROS params input:");
  ROS_INFO("<product_name>: %s", product_name.c_str());
  ROS_INFO("<topic_name>: %s", topic_name.c_str());
  ROS_INFO("<frame_id>: %s", setting.frame_id.c_str());
  ROS_INFO("<enable_serial_or_network_communication>: %s", 
    (enable_serial_or_network_communication?"Enable serial":"Enable network"));
  ROS_INFO("<port_name>: %s", port_name.c_str());
  ROS_INFO("<port_baudrate>: %d", serial_port_baudrate);
  ROS_INFO("<server_ip>: %s", server_ip.c_str());
  ROS_INFO("<server_port>: %s", server_port.c_str());
  ROS_INFO("<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  ROS_INFO("<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  ROS_INFO("<angle_crop_min>: %f", setting.angle_crop_min);
  ROS_INFO("<angle_crop_max>: %f", setting.angle_crop_max);
  ROS_INFO("<measure_point_freq>: %d", setting.measure_point_freq);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06; 
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "LDLiDAR_STL06P") {
    type_name = ldlidar::LDType::STL_06P;
  } else if (product_name == "LDLiDAR_STL27L") {
    type_name = ldlidar::LDType::STL_27L;
  } else if (product_name == "LDLiDAR_STL26") {
    type_name = ldlidar::LDType::STL_26;
  } else {
    ROS_ERROR("Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); //   注册时间戳获取函数

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (enable_serial_or_network_communication) {
    if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
      ROS_INFO("ldldiar node start is success");
    } else {
      ROS_ERROR("ldlidar node start is fail");
      exit(EXIT_FAILURE);
    }
  } else {
    if (ldlidarnode->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE)) {
      ROS_INFO("ldlidar node start is success");
    } else {
      ROS_ERROR("ldlidar node start is fail");
      exit(EXIT_FAILURE);
    }
  }

  if (ldlidarnode->WaitLidarCommConnect(500)) {
    ROS_INFO("ldlidar communication is normal.");
  } else {
    ROS_ERROR("ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  ros::Publisher lidar_pub = 
    nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic

  ros::Rate r(10); //10hz
  ldlidar::Points2D laser_scan_points;
  double lidar_spin_freq;
  bool is_get = false;
  while (ros::ok()) {
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1000)){
      case ldlidar::LidarStatus::NORMAL: 
        if (!is_get) {
          is_get = true;
          ROS_INFO("get ldlidar normal data and publish topic message.");
        }
        ldlidarnode->GetLidarSpinFreq(lidar_spin_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_spin_freq, setting, lidar_pub);
        break;
      case ldlidar::LidarStatus::ERROR:
        ROS_ERROR("ldlidar driver error.");
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        ROS_ERROR("get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    r.sleep();
  }

  ldlidarnode->Stop();

  delete ldlidarnode;
  ldlidarnode = nullptr;
  
  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
  LaserScanSetting& setting, ros::Publisher& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }
  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 30;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size - 1);

  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (float)(beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          ROS_ERROR("error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
              index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub.publish(output);
    end_scan_time = start_scan_time;
  } 
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
