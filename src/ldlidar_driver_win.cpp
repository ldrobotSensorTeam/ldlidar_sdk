/**
 * @file ldlidar_driver.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar sdk interface
 *         This code is only applicable to LDROBOT LiDAR products 
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2023-03
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
#include "ldlidar_driver/ldlidar_driver_win.h"

namespace ldlidar {

LDLidarDriverWinInterface::LDLidarDriverWinInterface() : 
  comm_pkg_(new LdLidarDataProcess()),
  comm_port_handle_(new SerialInterfaceWin()) {
  
  last_pubdata_times_ = std::chrono::steady_clock::now();
}

LDLidarDriverWinInterface::~LDLidarDriverWinInterface() {
  if (comm_pkg_ != nullptr) {
    delete comm_pkg_;
  }

  if (comm_port_handle_ != nullptr) {
    delete comm_port_handle_;
  }
}

bool LDLidarDriverWinInterface::Connect(LDType product_name, 
  std::string& serial_port_name, 
  PortParams& port_params) {

  if (is_connect_flag_) {
    return true;
  }

  if (serial_port_name.empty()) {
    LOG_ERROR("input <serial_port_name> is empty.","");
    return false;
  }

  if (register_get_timestamp_handle_ == nullptr) {
    LOG_ERROR("get timestamp fuctional is not register.","");
    return false;
  }

  comm_pkg_->ClearDataProcessStatus();
  comm_pkg_->RegisterTimestampGetFunctional(register_get_timestamp_handle_);
  comm_pkg_->SetProductType(product_name);

  // Win32
  // 设置串口参数
  comm_port_handle_->setPortParams(port_params);
  // 设置串口接收回调函数
  comm_port_handle_->setReadCallback(std::bind(
    &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
  // 打开串口
  if (!comm_port_handle_->open(serial_port_name)) {
    LOG_ERROR("serial is not open:%s", serial_port_name.c_str());
    return false;
  }

  is_connect_flag_ = true;

  SetLidarDriverStatus(true);

  return true;
}

bool LDLidarDriverWinInterface::Disconnect(void)  {
  if (!is_connect_flag_) {
    return true;
  }

  SetLidarDriverStatus(false);

  comm_port_handle_->close();
  
  is_connect_flag_ = false;
  
  return true;
}

void LDLidarDriverWinInterface::EnablePointCloudDataFilter(bool is_enable) {
  comm_pkg_->SetNoiseFilter(is_enable);
}

bool LDLidarDriverWinInterface::WaitLidarComm(int64_t timeout) {
  if (!is_connect_flag_) {
    return false;
  }

  auto last_time = std::chrono::steady_clock::now();

  bool is_recvflag = false;
  do {
    if (comm_pkg_->GetLidarPowerOnCommStatus()) {
      is_recvflag = true;
    }
    Sleep(1);
  } while (!is_recvflag && (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_time).count() < timeout));

  if (is_recvflag) {
    SetLidarDriverStatus(true);
    return true;
  } else {
    SetLidarDriverStatus(false);
    return false;
  }
}

LidarStatus LDLidarDriverWinInterface::GetLaserScanData(Points2D& dst, int64_t timeout) {
  if (!is_start_flag_) {
    return LidarStatus::STOP;
  }

  LidarStatus status = comm_pkg_->GetLidarStatus();
  if (LidarStatus::NORMAL == status) {
    if (comm_pkg_->GetLaserScanData(dst)) {
      last_pubdata_times_ = std::chrono::steady_clock::now(); 
      return LidarStatus::NORMAL;
    }
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_pubdata_times_).count() > timeout) {
      return LidarStatus::DATA_TIME_OUT;
    } else {
      return LidarStatus::DATA_WAIT;
    }
  } else {
    last_pubdata_times_ = std::chrono::steady_clock::now(); 
    return status;
  }
}

LidarStatus LDLidarDriverWinInterface::GetLaserScanData(LaserScan& dst, int64_t timeout) {
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
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_pubdata_times_).count() > timeout) {
      return LidarStatus::DATA_TIME_OUT;
    } else {
      return LidarStatus::DATA_WAIT;
    }
  } else {
    last_pubdata_times_ = std::chrono::steady_clock::now(); 
    return status;
  }
}

bool  LDLidarDriverWinInterface::GetLidarScanFreq(double& spin_hz) {
  if (!is_start_flag_) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();
  return true;
}

void LDLidarDriverWinInterface::RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) {
  register_get_timestamp_handle_ = get_timestamp_handle;
}

uint8_t LDLidarDriverWinInterface::GetLidarErrorCode(void) {
  if (!is_start_flag_) {
    return LIDAR_NO_ERROR;
  }
  
  uint8_t errcode = comm_pkg_->GetLidarErrorCode();
  return errcode;
}

bool LDLidarDriverWinInterface::Start(void) {
  if (is_start_flag_) {
    return true;
  }

  if (!is_connect_flag_) {
    return false;
  }

  is_start_flag_ = true;

  last_pubdata_times_ = std::chrono::steady_clock::now();

  SetLidarDriverStatus(true);

  return true;
}

bool LDLidarDriverWinInterface::Stop(void) {
  if (!is_start_flag_) {
    return true;
  }

  SetLidarDriverStatus(false);
  
  is_start_flag_ = false;
  
  return true;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/