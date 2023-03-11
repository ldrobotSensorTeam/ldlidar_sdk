/**
 * @file serial_interface_win.cpp
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

#include "ldlidar_driver/serial_interface_win.h"
#include "ldlidar_driver/log_module.h"

#include <Cfgmgr32.h>
#include <SetupAPI.h>
#include <tchar.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <map>

#pragma comment(lib, "SetupAPI.lib")
#pragma comment(lib, "Cfgmgr32.lib")

using namespace std;

SerialInterfaceWin::SerialInterfaceWin()
    : mIsOpened(false),
      mByteToRead(4096),
      mRxCounter(0),
      mTxCounter(0),
      commErrorHandle(nullptr),
      readCallback(nullptr),
      mRxThread(nullptr),
      mMaxBuffSize(4096) {
  mComHandle = INVALID_HANDLE_VALUE;
  memset(&mOverlappedSend, 0, sizeof(OVERLAPPED));
  memset(&mOverlappedRecv, 0, sizeof(OVERLAPPED));
  mOverlappedSend.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
  mOverlappedRecv.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
}

SerialInterfaceWin::~SerialInterfaceWin() {
  if (mOverlappedRecv.hEvent != INVALID_HANDLE_VALUE) {
    CloseHandle(mOverlappedRecv.hEvent);
    mOverlappedRecv.hEvent = INVALID_HANDLE_VALUE;
  }
  if (mOverlappedSend.hEvent != INVALID_HANDLE_VALUE) {
    CloseHandle(mOverlappedSend.hEvent);
    mOverlappedSend.hEvent = INVALID_HANDLE_VALUE;
  }
}

std::string wcharToChar(const wchar_t *wp, UINT encode = CP_ACP) {
  std::string str;
  int len =
      WideCharToMultiByte(encode, 0, wp, (int)wcslen(wp), NULL, 0, NULL, NULL);
  char *m_char = new char[len + 1];
  WideCharToMultiByte(encode, 0, wp, (int)wcslen(wp), m_char, len, NULL, NULL);
  m_char[len] = '\0';
  str = m_char;
  delete m_char;
  return str;
}

bool SerialInterfaceWin::availablePorts(vector<PortInfo> &availabelPortInfo) {
  DWORD dwGuids = 0;
  TCHAR propBuf[1024];
  PortInfo portInfo;

  availabelPortInfo.clear();

  SetupDiClassGuidsFromName(_T("Ports"), NULL, 0, &dwGuids);
  if (dwGuids == 0) return false;

  GUID *pGuids = new GUID[dwGuids];
  SetupDiClassGuidsFromName(_T("Ports"), pGuids, dwGuids, &dwGuids);

  for (DWORD i = 0; i < dwGuids; i++) {
    HDEVINFO hDevInfo =
        SetupDiGetClassDevs(&pGuids[i], NULL, NULL, DIGCF_PRESENT);
    if (hDevInfo == INVALID_HANDLE_VALUE) break;

    for (int index = 0;; index++) {
      SP_DEVINFO_DATA devInfoData;

      devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

      if (!SetupDiEnumDeviceInfo(hDevInfo, index, &devInfoData)) {
        break;
      }
      propBuf[0] = 0;

      CM_Get_Device_ID(devInfoData.DevInst, propBuf, 1024, 0);

      string s(propBuf);

      string wusb = s.substr(0, 3);

      if (wusb == "USB") {
        portInfo.vid = s.substr(8, 4);
        portInfo.pid = s.substr(17, 4);
      } else {
        portInfo.pid = "None";
        portInfo.vid = "None";
      }

      if (!SetupDiGetDeviceRegistryProperty(hDevInfo, &devInfoData,
                                            SPDRP_DEVICEDESC, NULL,
                                            (PBYTE)propBuf, MAX_PATH - 1, NULL))
        continue;

      portInfo.description = propBuf;

      HKEY hDeviceKey =
          SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0,
                              DIREG_DEV, KEY_QUERY_VALUE);
      if (hDeviceKey) {
        propBuf[0] = 0;
        DWORD dw_size = sizeof(propBuf);
        DWORD dw_type = 0;

        if ((RegQueryValueEx(hDeviceKey, _T("PortName"), NULL, &dw_type,
                            (LPBYTE)propBuf, &dw_size) == ERROR_SUCCESS) &&
            (dw_type == REG_SZ)) {
          portInfo.name = propBuf;
          availabelPortInfo.push_back(portInfo);
        }
      }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
  }
  delete[] pGuids;

  return true;
}

bool SerialInterfaceWin::open(string portName) {
  if (mIsOpened) close();

  portParams.portName = portName;

  std::string name = "\\\\.\\" + portName;

  mComHandle = ::CreateFile(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                            OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);

  if (mComHandle == INVALID_HANDLE_VALUE) {
    CloseHandle(mComHandle);
    return false;
  }

  PurgeComm(mComHandle,
            PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
  if (SetupComm(mComHandle, (DWORD)mMaxBuffSize, (DWORD)mMaxBuffSize) == 0) {
    CloseHandle(mComHandle);
    return false;
  }

  DCB dcb;
  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(mComHandle, &dcb) == 0) {
    CloseHandle(mComHandle);
    return false;
  } else {
    dcb.BaudRate = (DWORD)portParams.baudrate;
    dcb.ByteSize = (BYTE)portParams.dataBits;
    dcb.Parity = (BYTE)portParams.parity;
    dcb.StopBits = (BYTE)portParams.stopBits;
    if (portParams.flowControl == FlowControl::SoftwareControl) {
      dcb.fOutX = 1;
      dcb.fInX = 1;
    } else if (portParams.flowControl == FlowControl::HardwareControl) {
      dcb.fRtsControl = 0;
      dcb.fDtrControl = 1;
    } else {
      dcb.fOutX = 0;
      dcb.fInX = 0;
      dcb.fRtsControl = 0;
      dcb.fDtrControl = 0;
    }

    if (SetCommState(mComHandle, &dcb) == 0) {
      CloseHandle(mComHandle);
      return false;
    }
  }

  COMMTIMEOUTS timeouts;
  timeouts.ReadIntervalTimeout = 2;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 5;
  timeouts.WriteTotalTimeoutConstant = 50000;
  timeouts.WriteTotalTimeoutMultiplier = 1000;
  SetCommTimeouts(mComHandle, &timeouts);

  mIsOpened = true;

  mRxThreadRunFlag = true;
  mRxThread = new std::thread(rxThreadProc, this);

  return true;
}

bool SerialInterfaceWin::close() {
  if (!mIsOpened) return true;

  mIsOpened = false;
  mRxThreadRunFlag = false;

  if (mComHandle != INVALID_HANDLE_VALUE) {
    CloseHandle(mComHandle);
    mComHandle = INVALID_HANDLE_VALUE;
  }

  try {
    if (mRxThread->joinable()) mRxThread->join();
  } catch (const std::system_error &e) {
    LOG_INFO("Caught system_error with code:%d, meaning:%s", e.code(), e.what());
  }
  delete mRxThread;
  mRxThread = nullptr;

  return true;
}

int SerialInterfaceWin::readAll() { return 0; }

bool SerialInterfaceWin::write(const char *txBuf, uint32_t txBufLen) {
  DWORD txLen = 0;
  if (mIsOpened) {
    ResetEvent(mOverlappedSend.hEvent);
    uint32_t res = WriteFile(mComHandle, txBuf, txBufLen, (LPDWORD)&txLen,
                            &mOverlappedSend);
    if (!res) {
      if (GetLastError() == ERROR_IO_PENDING) {
        WaitForSingleObject(mOverlappedSend.hEvent, INFINITE);
        GetOverlappedResult(mComHandle, &mOverlappedSend, (LPDWORD)&txLen,
                            FALSE);
        mTxCounter += txLen;
        return true;
      }
    }
  }

  return false;
}

void SerialInterfaceWin::setPortParams(PortParams params) {
  memcpy(&portParams, &params, sizeof(params));
}

PortParams SerialInterfaceWin::currPortParams(void) { return portParams; }

void SerialInterfaceWin::rxThreadProc(SerialInterfaceWin *pClass) {
  char *rxBuf = new char[pClass->mMaxBuffSize + 1];
  ResetEvent(pClass->mOverlappedRecv.hEvent);
  LOG_INFO("rx thread start...","");

  map<long, string> error_code{
      {ERROR_INVALID_PARAMETER, "Invalid parameter"},
      {ERROR_ACCESS_DENIED, "Access denied"},
      {ERROR_INVALID_HANDLE, "Open failed"},
      {ERROR_BAD_COMMAND, "Illegal disconnection"},
  };

  while (pClass->mRxThreadRunFlag) {
    uint32_t readed = 0;
    bool res = ReadFile(pClass->mComHandle, rxBuf, (DWORD)pClass->mByteToRead,
                        (LPDWORD)&readed, &pClass->mOverlappedRecv);
    if (res == FALSE) {
      long last_error = GetLastError();
      switch (last_error) {
        case ERROR_IO_PENDING: {
          if (WaitForSingleObject(
            pClass->mOverlappedRecv.hEvent, INFINITE) == WAIT_OBJECT_0) {
            GetOverlappedResult(pClass->mComHandle, &pClass->mOverlappedRecv, (LPDWORD)&readed, FALSE);
            if (readed) {
              pClass->mRxCounter += readed;
              // callback
              if (pClass->readCallback != nullptr) {
                pClass->readCallback(rxBuf, readed);
              }
            }
          }
          break;
        }
        case ERROR_INVALID_PARAMETER:  /* 系统错误 erroe code:87 */ 
        case ERROR_ACCESS_DENIED:      /* 拒绝访问 erroe code:5 */
        case ERROR_INVALID_HANDLE:     /* 打开串口失败 erroe code:6 */ 
        case ERROR_BAD_COMMAND:       /* 连接过程中非法断开 erroe code:22 */ 
        {
          /* 不能再这里及回调函数中调用close，因为close中有join。在当前线程中join自己，会造成死锁 */ 
          if (pClass->commErrorHandle != nullptr)
            pClass->commErrorHandle(error_code[last_error]);
          pClass->mRxThreadRunFlag = false;

          break;
        }

        default:  /*  发生其他错误，其中有串口读写中断开串口连接的错误（错误22） */ 
        {
          if (pClass->commErrorHandle != nullptr)
            pClass->commErrorHandle("Unknow error");
          break;
        }
      }

      ResetEvent(pClass->mOverlappedRecv.hEvent);
    }
  }

  delete[] rxBuf;
}
/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/
