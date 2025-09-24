/**
 * @file cmd_interface_linux.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  linux serial port App
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

#ifndef __LINUX_SERIAL_PORT_H__
#define __LINUX_SERIAL_PORT_H__

#include <inttypes.h>
#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
namespace asmtermios {
#include <linux/termios.h>
}
#include <termios.h>
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

class SerialInterfaceLinux {
 public:
  SerialInterfaceLinux();
  ~SerialInterfaceLinux();
  // open serial port
  bool Open(std::string &port_name, uint32_t com_baudrate);  
  // close serial port
  bool Close();     
  // receive from port channel data                  
  bool ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len, uint32_t *rx_len); 
  // transmit data to port channel
  bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len, uint32_t *tx_len);  
  // set receive port channel data callback deal with fuction
  void SetReadCallback(std::function<void(const char *, size_t length)> callback) {
    read_callback_ = callback;
  }  
  // whether open
  bool IsOpened() { return is_cmd_opened_.load(); };  

 private:
  std::thread *rx_thread_;
  long long rx_count_;
  int32_t com_handle_;
  uint32_t com_baudrate_;
  std::atomic<bool> is_cmd_opened_, rx_thread_exit_flag_;
  std::function<void(const char *, size_t length)> read_callback_;
  static void RxThreadProc(void *param);
};

} // namespace ldlidar

#endif  //__LINUX_SERIAL_PORT_H__

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/