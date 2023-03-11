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
#include "ldlidar_driver/ldlidar_driver_linux.h"

namespace ldlidar {

LDLidarDriverLinuxInterface::LDLidarDriverLinuxInterface() :  comm_pkg_(new LdLidarDataProcess()),
  comm_serial_(new SerialInterfaceLinux()),
  comm_tcp_network_(new TCPSocketInterfaceLinux()),
  comm_udp_network_(new UDPSocketInterfaceLinux()){
  
  last_pubdata_times_ = std::chrono::steady_clock::now();
}

LDLidarDriverLinuxInterface::~LDLidarDriverLinuxInterface() {
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

bool LDLidarDriverLinuxInterface::Connect(LDType product_name, 
  std::string serial_port_name, 
  uint32_t serial_baudrate,
  CommunicationModeType comm_mode) {

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

  if (COMM_SERIAL_MODE == comm_mode) {
    comm_serial_->SetReadCallback(std::bind(
      &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
    if (!comm_serial_->Open(serial_port_name, serial_baudrate)) {
      LOG_ERROR("serial is not open:%s", serial_port_name.c_str());
      return false;
    }
  } else {
    LOG_ERROR("input <comm_mode> value is not \"ldlidar::COMM_SERIAL_MODE\"","");
    return false;
  }

  is_connect_flag_ = true;

  SetLidarDriverStatus(true);

  return true;
}

bool LDLidarDriverLinuxInterface::Connect(LDType product_name, 
            const char* server_ip, 
            const char* server_port,
            CommunicationModeType comm_mode) {

  if (is_connect_flag_) {
    return true;
  }

  if (LDType::NO_VER == product_name) {
    LOG_ERROR("input <product_name> is abnormal.","");
    return false;
  }

  if ((server_ip == nullptr) || (server_port == nullptr)) {
    LOG_ERROR("input server_ip or server_port is null","");
    return false;
  }

  if ((COMM_NO_NULL == comm_mode) || (COMM_SERIAL_MODE == comm_mode)) {
    LOG_ERROR("input comm_mode param is error.","");
    return false;
  }

  if (register_get_timestamp_handle_ == nullptr) {
    LOG_ERROR("get timestamp fuctional is not register.","");
    return false;
  }

  comm_pkg_->ClearDataProcessStatus();
  comm_pkg_->RegisterTimestampGetFunctional(register_get_timestamp_handle_);
  comm_pkg_->SetProductType(product_name);

  switch (comm_mode) {
    case COMM_TCP_CLIENT_MODE: {
      comm_tcp_network_->SetRecvCallback(std::bind(
        &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_tcp_network_->CreateSocket(TCP_CLIENT, server_ip, server_port);
      if (!result) {
        LOG_ERROR("client host: create socket is fail.","");
        return false;
      }
      LOG_INFO("client host: create socket is ok.","");
    }
      break;
    case COMM_TCP_SERVER_MODE: {
      comm_tcp_network_->SetRecvCallback(std::bind(
        &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_tcp_network_->CreateSocket(TCP_SERVER, server_ip, server_port);
      if (!result) {
        LOG_ERROR("server host: create socket is fail.","");
        return false;
      }
      LOG_INFO("server host: create socket is ok.","");
    }
      break;
    case COMM_UDP_CLIENT_MODE: {
      comm_udp_network_->SetRecvCallback(std::bind(
        &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2));
      bool result = comm_udp_network_->CreateSocket(UDP_CLIENT, server_ip, server_port);
      if (!result) {
        LOG_ERROR("client host: create socket is fail.","");
        return false;
      }
      // 主动向服务端发布消息使服务端保存客户端ip，port 信息，建立沟通渠道
      uint8_t trans_buf[4] = {0xa5, 0x5a, 0x00, 0x00};
      uint32_t tx_len;
      if (!comm_udp_network_->TransToNet((uint8_t *)trans_buf, sizeof(trans_buf), &tx_len)) {
        LOG_ERROR("client host: send request to server is fail. %s", strerror(errno));
        return false;
      }
      LOG_INFO("client host: create socket is ok.","");
    }
      break;
    case COMM_UDP_SERVER_MODE: {
      comm_udp_network_->SetRecvCallback(std::bind(
        &LdLidarDataProcess::CommReadCallback, comm_pkg_, std::placeholders::_1, std::placeholders::_2)); 
      bool result = comm_udp_network_->CreateSocket(UDP_SERVER, server_ip, server_port);
      if (!result) {
        LOG_ERROR("server host: create socket is fail.","");
        return false;
      }
      LOG_INFO("server host: create socket is ok.","");
      LOG_INFO("server host: wait client ack connect..","");
      while (!comm_udp_network_->IsClientAck()) {
        usleep(1000);
      }
    }
      break;
    default: {
      LOG_ERROR("input comm_mode param is error.","");
      return false;
    }
      break;
  }
  
  is_connect_flag_ = true;

  SetLidarDriverStatus(true);
  
  return true;
}

bool LDLidarDriverLinuxInterface::Disconnect(void)  {
  if (!is_connect_flag_) {
    return true;
  }

  SetLidarDriverStatus(false);

  comm_serial_->Close();
  comm_tcp_network_->CloseSocket();
  comm_udp_network_->CloseSocket();
  
  is_connect_flag_ = false;
  
  return true;
}

void LDLidarDriverLinuxInterface::EnablePointCloudDataFilter(bool is_enable) {
  comm_pkg_->SetNoiseFilter(is_enable);
}

bool LDLidarDriverLinuxInterface::WaitLidarComm(int64_t timeout) {
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
    SetLidarDriverStatus(true);
    return true;
  } else {
    SetLidarDriverStatus(false);
    return false;
  }
}

LidarStatus LDLidarDriverLinuxInterface::GetLaserScanData(Points2D& dst, int64_t timeout) {
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

LidarStatus LDLidarDriverLinuxInterface::GetLaserScanData(LaserScan& dst, int64_t timeout) {
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

bool  LDLidarDriverLinuxInterface::GetLidarScanFreq(double& spin_hz) {
  if (!is_start_flag_) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();
  return true;
}

void LDLidarDriverLinuxInterface::RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) {
  register_get_timestamp_handle_ = get_timestamp_handle;
}

uint8_t LDLidarDriverLinuxInterface::GetLidarErrorCode(void) {
  if (!is_start_flag_) {
    return LIDAR_NO_ERROR;
  }
  
  uint8_t errcode = comm_pkg_->GetLidarErrorCode();
  return errcode;
}

bool LDLidarDriverLinuxInterface::Start(void) {
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

bool LDLidarDriverLinuxInterface::Stop(void) {
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