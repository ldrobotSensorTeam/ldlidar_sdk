/**
 * @file serial_interface_win.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  Win32 serial port App
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2017-2023  SHENZHEN LDROBOT CO., LTD. All rights
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

#pragma once
#include <Windows.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

struct PortInfo {
  std::string name;
  std::string description;
  std::string pid;
  std::string vid;
};

enum class BaudRate {
  Baud1200 = 1200,
  Baud2400 = 2400,
  Baud4800 = 4800,
  Baud9600 = 9600,
  Baud19200 = 19200,
  Baud38400 = 38400,
  Baud57600 = 57600,
  Baud115200 = 115200,
  Baud230400 = 230400,
  Unknonw = -1
};
enum class DataBits {
  Data5 = 5,
  Data6 = 6,
  Data7 = 7,
  Data8 = 8,
  Unknonw = -1
};
enum class FlowControl {
  NoFlowControl,
  HardwareControl,
  SoftwareControl,
  Unknonw = -1
};
enum class Parity {
  NoParity,
  EvenParity,
  OddParity,
  SpaceParity,
  MarkParity,
  Unknonw = -1
};
enum class StopBits { OneStop = 0, TwoStop, OneAndHalfStop, Unknow };

struct PortParams {
  std::string portName;
  BaudRate baudrate = BaudRate::Baud115200;
  Parity parity = Parity::NoParity;
  DataBits dataBits = DataBits::Data8;
  StopBits stopBits = StopBits::OneStop;
  FlowControl flowControl = FlowControl::NoFlowControl;
};

class SerialInterfaceWin {
 public:
  SerialInterfaceWin();
  ~SerialInterfaceWin();

  static bool availablePorts(std::vector<PortInfo> &availabelPortInfo);
  bool open(std::string port_name);
  bool close();
  bool write(const char *tx_buf, uint32_t tx_buf_len);
  int readAll();
  long long rxLength() { return mRxCounter; }
  long long txLength() { return mTxCounter; }
  void clearRxLength(void) { mRxCounter = 0; }
  bool isOpen() { return mIsOpened; }
  void setBufferSize(size_t size) { mMaxBuffSize = size; }
  void setCommErrorCallback(std::function<void(std::string)> callback) {
    commErrorHandle = callback;
  }
  void setReadCallback(
      std::function<void(const char *, size_t length)> callback) {
    readCallback = callback;
  }

  void setPortParams(PortParams params);
  PortParams currPortParams(void);

 private:
  bool mIsOpened;
  HANDLE mComHandle;
  std::thread *mRxThread;
  static void rxThreadProc(SerialInterfaceWin *pClass);
  size_t mByteToRead;
  long long mRxCounter;
  long long mTxCounter;
  size_t mMaxBuffSize;
  OVERLAPPED mOverlappedSend;
  OVERLAPPED mOverlappedRecv;
  std::atomic<bool> mRxThreadRunFlag;

  std::function<void(std::string)> commErrorHandle;
  std::function<void(const char *, size_t length)> readCallback;

  PortParams portParams;
};

/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/