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

#include "ldlidar_driver.h"

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

int run_lidar_node(int argc, char **argv) {

  if (argc < 4) {
    LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> serialcom <serial_number>","");
    LDS_LOG_WARN("or","");
    LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> networkcom_tcpclient <server_ip> <server_port>","");
    LDS_LOG_WARN("For example:","");
    LDS_LOG_WARN("./ldlidar_stl LD06 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl LD19 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL06P serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL27L serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL26 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("-----------------","");
    LDS_LOG_WARN("./ldlidar_stl LD06 networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl LD19 networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL06P networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL27L networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL26 networkcom_tcpclient 192.168.1.200 2000","");
    exit(EXIT_FAILURE);
  }

  std::string product_name(argv[1]);
  std::string communication_mode(argv[2]);
  std::string port_name;
  std::string server_ip;
  std::string server_port;
  uint32_t serial_baudrate = 0;
  ldlidar::LDType type_name;

  if ((communication_mode != "serialcom") && 
      (communication_mode != "networkcom_tcpclient")) {
    LD_LOG_ERROR("Terminal:input argc(3) value is error","");
    LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> serialcom <serial_number>","");
    LDS_LOG_WARN("or","");
    LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> neworkcom_tcpclient <server_ip> <server_port>","");
    LDS_LOG_WARN("For example:","");
    LDS_LOG_WARN("./ldlidar_stl LD06 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl LD19 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL06P serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL27L serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("./ldlidar_stl STL26 serialcom /dev/ttyUSB0","");
    LDS_LOG_WARN("-----------------","");
    LDS_LOG_WARN("./ldlidar_stl LD06 networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl LD19 networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL06P networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL27L networkcom_tcpclient 192.168.1.200 2000","");
    LDS_LOG_WARN("./ldlidar_stl STL26 networkcom_tcpclient 192.168.1.200 2000","");
    exit(EXIT_FAILURE);
  } else {
    if (communication_mode == "serialcom") {
      if (argc != 4) {
        LD_LOG_ERROR("Terminal:input argc != 4","");
        exit(EXIT_FAILURE);
      }
      port_name = argv[3];
    } else if (communication_mode == "networkcom_tcpclient") {
      if (argc != 5) {
        LD_LOG_ERROR("Terminal:input argc != 5","");
        exit(EXIT_FAILURE);
      }
      server_ip = argv[3];
      server_port = argv[4];
    }
  }

  ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();

  LDS_LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

  node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); //   注册时间戳获取函数

  node->EnableFilterAlgorithnmProcess(true);

  if (product_name == "LD06") {
    serial_baudrate = 230400;
    type_name = ldlidar::LDType::LD_06; 
  } else if (product_name == "LD19") {
    serial_baudrate = 230400;
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "STL06P") {
    serial_baudrate = 230400;
    type_name = ldlidar::LDType::STL_06P;
  } else if (product_name == "STL27L") {
    serial_baudrate = 921600;
    type_name = ldlidar::LDType::STL_27L;
  }else if (product_name == "STL26") {
    serial_baudrate = 230400;
    type_name = ldlidar::LDType::STL_26;
  } else {
    LDS_LOG_ERROR("input <product_name> is error!","");
    exit(EXIT_FAILURE);
  }

  if (communication_mode == "serialcom") {
    if (node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE)) {
      LDS_LOG_INFO("ldlidar node start is success","");
    } else {
      LD_LOG_ERROR("ldlidar node start is fail","");
      exit(EXIT_FAILURE);
    }
  } else if (communication_mode == "networkcom_tcpclient") {
    if (node->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE)) {
      LDS_LOG_INFO("ldlidar node start is success","");
    } else {
      LD_LOG_ERROR("ldlidar node start is fail","");
      exit(EXIT_FAILURE);
    }
  }

  if (node->WaitLidarCommConnect(500)) {
    LDS_LOG_INFO("ldlidar communication is normal.","");
  } else {
    LDS_LOG_ERROR("ldlidar communication is abnormal.","");
    exit(EXIT_FAILURE);
  }

  ldlidar::Points2D laser_scan_points;
  double lidar_spin_freq;
  unsigned int count = 100;
  while (ldlidar::LDLidarDriver::IsOk()) {
    switch (node->GetLaserScanData(laser_scan_points, 1000)){
      case ldlidar::LidarStatus::NORMAL: 
        // get lidar normal data
        if (node->GetLidarSpinFreq(lidar_spin_freq)) {
          LDS_LOG_INFO("speed(Hz):%f", lidar_spin_freq);
        }
        //  对点云数据进行遍历输出
        for (auto point : laser_scan_points) {
          LDS_LOG_INFO("stamp(ns):%lld,angle:%f,distance(mm):%d,intensity:%d", 
            point.stamp, point.angle, point.distance, point.intensity);
        }
        LDS_LOG_INFO("size:%d,stamp_front:%lld, stamp_back:%lld",laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
        break;
      case ldlidar::LidarStatus::ERROR:
        LDS_LOG_ERROR("ldlidar is error.","");
        node->Stop();
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.","");
        node->Stop();
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    if (count-- <= 0) {
      ldlidar::LDLidarDriver::SetIsOkStatus(false);
    }

    usleep(1000 * 100);  // sleep 100ms  == 10Hz
  }

  node->Stop();

  delete node;
  node = nullptr;

  return 0;
}

int main(int argc, char** argv) {

  for (int i = 0; i < 10000; i++) {
    run_lidar_node(argc, argv);
  }
  
  while (1) {
    sleep(1);  
  }
  
  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/