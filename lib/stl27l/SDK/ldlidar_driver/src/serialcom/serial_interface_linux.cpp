/**
 * @file cmd_interface_linux.cpp
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

#include "serial_interface_linux.h"
#include "log_module.h"


#define MAX_ACK_BUF_LEN 4096

namespace ldlidar {

SerialInterfaceLinux::SerialInterfaceLinux()
    : rx_thread_(nullptr), rx_count_(0), read_callback_(nullptr) {
  com_handle_ = -1;
  com_baudrate_ = 0;
}

SerialInterfaceLinux::~SerialInterfaceLinux() { Close(); }

bool SerialInterfaceLinux::Open(std::string &port_name, uint32_t com_baudrate) {
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  com_handle_ = open(port_name.c_str(), flags);
  if (-1 == com_handle_) {
    LD_LOG_ERROR("Open open error,%s", strerror(errno));
    return false;
  }

  com_baudrate_ = com_baudrate;

  struct asmtermios::termios2 options;
  if (ioctl(com_handle_, _IOC(_IOC_READ, 'T', 0x2A, sizeof(struct asmtermios::termios2)), &options)) {
    LD_LOG_ERROR("TCGETS2 first error,%s", strerror(errno));
    if (com_handle_ != -1) {
      close(com_handle_);
      com_handle_ = -1;
    }
    return false;
  }

  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8);
  options.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                  ISIG | IEXTEN);  //|ECHOPRT
  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

  options.c_cflag &= ~CBAUD;
  options.c_cflag |= BOTHER;
  options.c_ispeed = this->com_baudrate_;
  options.c_ospeed = this->com_baudrate_;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (ioctl(com_handle_, _IOC(_IOC_WRITE, 'T', 0x2B, sizeof(struct asmtermios::termios2)), &options)) {
    LD_LOG_ERROR("TCSETS2 error,%s", strerror(errno));
    if (com_handle_ != -1) {
      close(com_handle_);
      com_handle_ = -1;
    }
    return false;
  }

  if (ioctl(com_handle_, _IOC(_IOC_READ, 'T', 0x2A, sizeof(struct asmtermios::termios2)), &options)) {
    LD_LOG_ERROR("TCGETS2 second error,%s", strerror(errno));
    if (com_handle_ != -1) {
      close(com_handle_);
      com_handle_ = -1;
    }
    return false;
  }

  LDS_LOG_INFO("Actual BaudRate reported:%d", options.c_ospeed);

  tcflush(com_handle_, TCIFLUSH);

  rx_thread_exit_flag_ = false;
  rx_thread_ = new std::thread(RxThreadProc, this);
  is_cmd_opened_ = true;

  return true;
}

bool SerialInterfaceLinux::Close() {
  if (is_cmd_opened_ == false) {
    return true;
  }

  rx_thread_exit_flag_ = true;

  if (com_handle_ != -1) {
    close(com_handle_);
    com_handle_ = -1;
  }

  if ((rx_thread_ != nullptr) && rx_thread_->joinable()) {
    rx_thread_->join();
    delete rx_thread_;
    rx_thread_ = nullptr;
  }

  is_cmd_opened_ = false;

  return true;
}

bool SerialInterfaceLinux::ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len,
                                   uint32_t *rx_len) {
  static timespec timeout = {0, (long)(100 * 1e6)};
  int32_t len = -1;

  if (IsOpened()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(com_handle_, &read_fds);
    int r = pselect(com_handle_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (r < 0) {
      // Select was interrupted
      if (errno == EINTR) {
        return false;
      }
    } else if (r == 0) {  // timeout
      return false;
    }

    if (FD_ISSET(com_handle_, &read_fds)) {
      len = (int32_t)read(com_handle_, rx_buf, rx_buf_len);
      if ((len != -1) && rx_len) {
        *rx_len = len;
      }
    }
  }
  return len == -1 ? false : true;
}

bool SerialInterfaceLinux::WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len,
                                  uint32_t *tx_len) {
  int32_t len = -1;

  if (IsOpened()) {
    len = (int32_t)write(com_handle_, tx_buf, tx_buf_len);
    if ((len != -1) && tx_len) {
      *tx_len = len;
    }
  }
  return len == -1 ? false : true;
}

void SerialInterfaceLinux::RxThreadProc(void *param) {
  SerialInterfaceLinux *cmd_if = (SerialInterfaceLinux *)param;
  char *rx_buf = new char[MAX_ACK_BUF_LEN + 1];
  while (!cmd_if->rx_thread_exit_flag_.load()) {
    uint32_t readed = 0;
    bool res = cmd_if->ReadFromIO((uint8_t *)rx_buf, MAX_ACK_BUF_LEN, &readed);
    if (res && readed) {
      cmd_if->rx_count_ += readed;
      if (cmd_if->read_callback_ != nullptr) {
        cmd_if->read_callback_(rx_buf, readed);
      }
    }
  }

  delete[] rx_buf;
}

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/