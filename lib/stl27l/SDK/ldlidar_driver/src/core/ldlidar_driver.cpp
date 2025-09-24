/**
 * @file ldlidar_driver.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar processing App
 *         This code is only applicable to LDROBOT LiDAR LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-05-12
 *
 * @copyright Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights
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

namespace ldlidar {

bool LDLidarDriver::is_ok_ = false;

LDLidarDriver::LDLidarDriver() 
  : sdk_version_number_("v3.0.5"),
    is_start_flag_(false),
    register_get_timestamp_handle_(nullptr),
    comm_pkg_(new LiPkg()),
    comm_serial_(new SerialInterfaceLinux()),
    comm_tcp_network_(new TCPSocketInterfaceLinux()),
    comm_udp_network_(new UDPSocketInterfaceLinux()) {
    last_pubdata_times_ = std::chrono::steady_clock::now();
}

LDLidarDriver::~LDLidarDriver() {
  if (comm_pkg_ != nullptr) {
    delete comm_pkg_;
  }

  if (comm_serial_ != nullptr) {
    delete comm_serial_;
  }

  if (comm_tcp_network_ != nullptr) {
    delete comm_tcp_network_;
  }

  if (comm_udp_network_ != nullptr) {
    delete comm_udp_network_;
  }
}

std::string LDLidarDriver::GetLidarSdkVersionNumber(void) {
  return sdk_version_number_;
}

bool LDLidarDriver::Start(LDType product_name, 
                          std::string serial_port_name, 
                          uint32_t serial_baudrate,
                          CommunicationModeTypeDef comm_mode) {
  if (is_start_flag_) {
    return true;
  }

  if (LDType::NO_VERSION == product_name) {
    LD_LOG_ERROR("input <product_name> is abnormal.","");
    return false;
  }

  if (serial_port_name.empty() || (0 == serial_baudrate)) {
    LD_LOG_ERROR("input serial param error.","");
    return false;
  }

  if (register_get_timestamp_handle_ == nullptr) {
    LD_LOG_ERROR("get timestamp fuctional is not register.","");
    return false;
  }

  comm_pkg_->ClearDataProcessStatus();
  comm_pkg_->RegisterTimestampGetFunctional(register_get_timestamp_handle_);
  comm_pkg_->SetProductType(product_name);

  if (COMM_SERIAL_MODE == comm_mode) {
    comm_serial_->SetReadCallback(std::bind(&LiPkg::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
    if (!(comm_serial_->Open(serial_port_name, serial_baudrate))) {
      LD_LOG_ERROR("serial is not open:%s", serial_port_name.c_str());
      return false;
    }
  } else {
    LD_LOG_ERROR("input <comm_mode> value is not \"ldlidar::COMM_SERIAL_MODE\"","");
    return false;
  }

  is_start_flag_ = true;

  SetIsOkStatus(true);

  return true;
}

bool LDLidarDriver::Start(LDType product_name, 
            const char* server_ip, 
            const char* server_port,
            CommunicationModeTypeDef comm_mode) {

  if (is_start_flag_) {
    return true;
  }

  if (LDType::NO_VERSION == product_name) {
    LD_LOG_ERROR("input <product_name> is abnormal.","");
    return false;
  }

  if ((server_ip == nullptr) || (server_port == nullptr)) {
    LD_LOG_ERROR("input server_ip or server_port is null","");
    return false;
  }

  if ((COMM_NO_NULL == comm_mode) || (COMM_SERIAL_MODE == comm_mode)) {
    LDS_LOG_ERROR("input comm_mode param is error.","");
    return false;
  }

  if (register_get_timestamp_handle_ == nullptr) {
    LD_LOG_ERROR("get timestamp fuctional is not register.","");
    return false;
  }

  comm_pkg_->ClearDataProcessStatus();
  comm_pkg_->RegisterTimestampGetFunctional(register_get_timestamp_handle_);
  comm_pkg_->SetProductType(product_name);

  switch (comm_mode) {
    case COMM_TCP_CLIENT_MODE: {
      comm_tcp_network_->SetRecvCallback(std::bind(&LiPkg::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_tcp_network_->CreateSocket(TCP_CLIENT, server_ip, server_port);
      if (!result) {
        LD_LOG_ERROR("client host: create socket is fail.","");
        return false;
      }
      LDS_LOG_INFO("client host: create socket is ok.","");
    }
      break;
    case COMM_TCP_SERVER_MODE: {
      comm_tcp_network_->SetRecvCallback(std::bind(&LiPkg::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_tcp_network_->CreateSocket(TCP_SERVER, server_ip, server_port);
      if (!result) {
        LD_LOG_ERROR("server host: create socket is fail.","");
        return false;
      }
      LDS_LOG_INFO("server host: create socket is ok.","");
    }
      break;
    case COMM_UDP_CLIENT_MODE: {
      comm_udp_network_->SetRecvCallback(std::bind(&LiPkg::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_udp_network_->CreateSocket(UDP_CLIENT, server_ip, server_port);
      if (!result) {
        LD_LOG_ERROR("client host: create socket is fail.","");
        return false;
      }
      // 主动向服务端发布消息使服务端保存客户端ip，port 信息，建立沟通渠道
      uint8_t trans_buf[4] = {0xa5, 0x5a, 0x00, 0x00};
      uint32_t tx_len;
      if (!comm_udp_network_->TransToNet((uint8_t *)trans_buf, sizeof(trans_buf), &tx_len)) {
        LD_LOG_ERROR("client host: send request to server is fail. %s", strerror(errno));
        return false;
      }
      LDS_LOG_INFO("client host: create socket is ok.","");
    }
      break;
    case COMM_UDP_SERVER_MODE: {
      comm_udp_network_->SetRecvCallback(std::bind(&LiPkg::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2)); 
      bool result = comm_udp_network_->CreateSocket(UDP_SERVER, server_ip, server_port);
      if (!result) {
        LD_LOG_ERROR("server host: create socket is fail.","");
        return false;
      }
      LDS_LOG_INFO("server host: create socket is ok.","");
      LDS_LOG_INFO("server host: wait client ack connect..","");
      while (!comm_udp_network_->IsClientAck()) {
        usleep(1000);
      }
    }
      break;
    default: {
      LDS_LOG_ERROR("input comm_mode param is error.","");
      return false;
    }
      break;
  }
  
  is_start_flag_ = true;

  SetIsOkStatus(true);
  
  return true;
}

bool LDLidarDriver::Stop(void) {
  if (!is_start_flag_) {
    return true;
  }

  comm_serial_->Close(); 
  comm_tcp_network_->CloseSocket();
  comm_udp_network_->CloseSocket();
  
  is_start_flag_ = false;

  SetIsOkStatus(false);

  return true;
}

bool LDLidarDriver::WaitLidarCommConnect(int64_t timeout) {
  auto last_time = std::chrono::steady_clock::now();

  bool is_recvflag = false;
  do {
    if (comm_pkg_->GetLidarPowerOnCommStatus()) {
      is_recvflag = true;
    }
    usleep(1000);
  } while (!is_recvflag && (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_time).count() < timeout));

  if (is_recvflag) {
    last_pubdata_times_ = std::chrono::steady_clock::now();
    return true;
  } else {
    return false;
  }
}

LidarStatus LDLidarDriver::GetLaserScanData(Points2D& dst, int64_t timeout) {
  if (!is_start_flag_) {
    return LidarStatus::STOP;
  }

  LidarStatus status = comm_pkg_->GetLidarStatus();
  if (LidarStatus::NORMAL == status) {
    if (comm_pkg_->GetLaserScanData(dst)) {
      last_pubdata_times_ = std::chrono::steady_clock::now(); 
      return LidarStatus::NORMAL;
    }
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_pubdata_times_).count() > timeout) {
      return LidarStatus::DATA_TIME_OUT;
    } else {
      return LidarStatus::DATA_WAIT;
    }
  } else {
    last_pubdata_times_ = std::chrono::steady_clock::now(); 
    return status;
  }
}

LidarStatus LDLidarDriver::GetLaserScanData(LaserScan& dst, int64_t timeout) {
  if (!is_start_flag_) {
    return LidarStatus::STOP;
  }

  LidarStatus status = comm_pkg_->GetLidarStatus();
  if (LidarStatus::NORMAL == status) {
    Points2D recvpcd;
    if (comm_pkg_->GetLaserScanData(recvpcd)) {
      last_pubdata_times_ = std::chrono::steady_clock::now(); 
      dst.stamp = recvpcd.front().stamp;
      dst.points = recvpcd;
      return LidarStatus::NORMAL;
    }
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_pubdata_times_).count() > timeout) {
      return LidarStatus::DATA_TIME_OUT;
    } else {
      return LidarStatus::DATA_WAIT;
    }
  } else {
    last_pubdata_times_ = std::chrono::steady_clock::now(); 
    return status;
  }
}

bool  LDLidarDriver::GetLidarSpinFreq(double& spin_hz) {
  if (!is_start_flag_) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();

  return true;
}

void LDLidarDriver::RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) {
  register_get_timestamp_handle_ = get_timestamp_handle;
}

void LDLidarDriver::EnableFilterAlgorithnmProcess(bool is_enable) {
  comm_pkg_->EnableFilter(is_enable);
}

// uint8_t LDLidarDriver::GetLidarErrorCode(void) {
//   if (!is_start_flag_) {
//     return 0x00;
//   }
//   uint8_t errcode = comm_pkg_->GetLidarErrorCode();
//   return errcode;
// }

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/