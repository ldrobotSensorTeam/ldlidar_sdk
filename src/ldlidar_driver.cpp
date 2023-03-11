/**
 * @file ldlidar_driver.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar sdk interface
 *         This code is only applicable to LDROBOT LiDAR products 
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-05-12
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
#include "ldlidar_driver/ldlidar_driver.h"

namespace ldlidar {

bool LDLidarDriver::is_ok_ = false;

LDLidarDriver::LDLidarDriver() : 
  is_start_flag_(false),
  is_connect_flag_(false),
  sdk_pack_version_(LDLiDAR_SDK_VERSION_NUMBER) {

}

LDLidarDriver::~LDLidarDriver() {

}

std::string LDLidarDriver::GetLidarSdkVersionNumber(void) {
  return sdk_pack_version_;
}

bool LDLidarDriver::Ok() {
  return is_ok_; 
}

void LDLidarDriver::SetLidarDriverStatus(bool status) {
  is_ok_ = status;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/