/**
 * @file ldlidar_node.cpp
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
#include "ldlidar_node.h"

namespace ldlidar {

LDlidarNode::LDlidarNode(LDVersion product_name, std::string serial_port_name) {
  product_name_ = product_name;
  serial_port_name_ = serial_port_name;
  comm_pkg_ = nullptr;
  comm_serial_ = nullptr;
  is_init_flag_ = false;
  is_start_flag = false;

  if (product_name_ == LDVersion::LD_14) {
    if (serial_port_name_.empty()) {
      fprintf(stderr, "[ldrobot] error: LDlidarNode create, input <serial_port_name> is fault.\n");
      exit(EXIT_FAILURE);
    } else {
      comm_pkg_ = new LiPkg(product_name_);
      comm_serial_ = new CmdInterfaceLinux(115200);
    }
  } else {
    fprintf(stderr, "[ldrobot] error: LDlidarNode create, input <product_name> is fault.\n");
    exit(EXIT_FAILURE);
  }
  is_init_flag_ = true;
}

LDlidarNode::~LDlidarNode() {
  if (comm_pkg_ != nullptr) {
    delete comm_pkg_;
  }

  if (comm_serial_ != nullptr) {
    delete comm_serial_;
  }
}

bool LDlidarNode::StartNode(void) {
  if (!is_init_flag_) {
    return false;
  }

  if (!(comm_serial_->Open(serial_port_name_))) {
    fprintf(stderr,"[ldrobot] serial is not open %s\n", serial_port_name_.c_str());
    return false;
  }

  comm_serial_->SetReadCallback(std::bind(&LiPkg::CommReadCallBack, comm_pkg_, std::placeholders::_1, std::placeholders::_2));

  is_start_flag = true;
  return true;
}

bool LDlidarNode::StopNode(void)  {
  if (!is_init_flag_) {
    return false;
  } 
  comm_serial_->Close();
  is_start_flag = false;
}

bool LDlidarNode::GetLaserScan(Points2D& output) {
  if (!is_init_flag_) {
    return false;
  } else if (!is_start_flag) {
    return false;
  }
  bool ret = comm_pkg_->GetLaserScanData(output);
  return ret;
}

bool  LDlidarNode::GetLidarSpinFreq(double& spin_hz) {
  if (!is_init_flag_) {
    return false;
  }else if (!is_start_flag) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();
  return true;
}

bool LDlidarNode::GetLidarWorkStatus(LidarStatus& status) {
  if (!is_init_flag_) {
    return false;
  }else if (!is_start_flag) {
    return false;
  }
  status = comm_pkg_->GetLidarStatus();
  return true;
}

}
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/