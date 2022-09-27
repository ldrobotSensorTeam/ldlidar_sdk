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

LDlidarNode::LDlidarNode(){
  comm_pkg_ = new LiPkg();
  comm_serial_ = new CmdInterfaceLinux();
  is_start_flag = false;

}

LDlidarNode::~LDlidarNode() {
  if (comm_pkg_ != nullptr) {
    delete comm_pkg_;
  }

  if (comm_serial_ != nullptr) {
    delete comm_serial_;
  }
}

bool LDlidarNode::StartNode(LDType product_name, std::string serial_port_name, bool to_right_hand, bool is_filter) {

  comm_pkg_->SetProductType(product_name);
  comm_pkg_->SetLaserScanDir(to_right_hand);
  comm_pkg_->SetNoiseFilter(is_filter);

  comm_serial_->SetReadCallback(std::bind(&LiPkg::CommReadCallBack, comm_pkg_, std::placeholders::_1, std::placeholders::_2));

  if (!(comm_serial_->Open(serial_port_name, 115200))) {
    std::cerr << "[ldrobot] serial is not open " << serial_port_name << std::endl;
    return false;
  }

  is_start_flag = true;
  return true;
}

bool LDlidarNode::StopNode(void)  {
  if (!is_start_flag) {
    return true;
  } 

  comm_serial_->Close();
  is_start_flag = false;
  
  return true;
}

bool LDlidarNode::GetLaserScan(Points2D& output) {
  if (!is_start_flag) {
    return false;
  }
  bool ret = comm_pkg_->GetLaserScanData(output);
  return ret;
}

bool  LDlidarNode::GetLidarSpinFreq(double& spin_hz) {
  if (!is_start_flag) {
    return false;
  }
  spin_hz = comm_pkg_->GetSpeed();
  return true;
}

bool LDlidarNode::GetLidarWorkStatus(LidarStatus& status) {
  if (!is_start_flag) {
    return false;
  }
  status = comm_pkg_->GetLidarStatus();
  return true;
}

std::string LDlidarNode::GetSdkVersionNum(void) {
  std::string ver = comm_pkg_->GetSdkPackVersionNum();
  return ver;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/