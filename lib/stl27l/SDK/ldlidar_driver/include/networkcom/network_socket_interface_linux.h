/**
 * @file network_socket_interface_linux.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  linux network App
 * @version 0.1
 * @date 2022-09-05
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

#ifndef __SOCKET_INTERFACE_LINUX_H__
#define __SOCKET_INTERFACE_LINUX_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <sys/types.h>	      
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ldlidar {

typedef enum NetworkCommDevEnum {
  NET_NULL,
  UDP_SERVER, 
  UDP_CLIENT,
  TCP_SERVER,
  TCP_CLIENT
}NetCommDevTypeDef;

class UDPSocketInterfaceLinux {
public:
  UDPSocketInterfaceLinux();
  
  ~UDPSocketInterfaceLinux();
  
  bool CreateSocket(NetCommDevTypeDef obj, const char *ip, const char *port);

  bool CloseSocket();

  bool TransToNet(uint8_t *tx_buf, uint32_t tx_buff_len, uint32_t *tx_len);

  void SetRecvCallback(std::function<void(const char *, size_t length)> callback);

  bool IsClientAck() { return is_server_recv_ack_flag_.load();}

private:
  std::thread *recv_thread_;
  long long recv_count_;
  int32_t com_sockfd_;
  NetCommDevTypeDef ncd_;
  std::atomic<bool> is_cmd_created_, recv_thread_exit_flag_, is_server_recv_ack_flag_;
  std::function<void(const char *, size_t length)> recv_callback_;
  std::string server_ip_, server_port_;
  std::string client_ip_, client_port_;

  bool IsCreated() { return is_cmd_created_.load(); }

  bool RecvFromNet(uint8_t *rx_buf , uint32_t rx_buff_len, uint32_t *rx_len);

  static void RecvThreadProc(void *param);
};


class TCPSocketInterfaceLinux {
public:
  TCPSocketInterfaceLinux();
  
  ~TCPSocketInterfaceLinux();
  
  bool CreateSocket(NetCommDevTypeDef obj, const char *ip, const char *port);

  bool CloseSocket();

  bool TransToNet(uint8_t *tx_buf, uint32_t tx_buff_len, uint32_t *tx_len);

  void SetRecvCallback(std::function<void(const char *, size_t length)> callback);

  

private:
  std::thread *recv_thread_;
  long long recv_count_;
  int32_t com_sockfd_;
  int32_t listend_client_sockfd_;   //// server model used
  NetCommDevTypeDef ncd_;
  std::atomic<bool> is_cmd_created_, recv_thread_exit_flag_;
  std::function<void(const char *, size_t length)> recv_callback_;

  bool IsCreated() { return is_cmd_created_.load(); }

  bool RecvFromNet(uint8_t *rx_buf , uint32_t rx_buff_len, uint32_t *rx_len);
  
  static void RecvThreadProc(void *param);
};

} // namespace ldlidar
#endif //  __SOCKET_INTERFACE_LINUX_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/