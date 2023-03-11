/**
 * @file ldlidar_driver.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar sdk interface
 *         This code is only applicable to LDROBOT LiDAR LD14
 * products sold by Shenzhen LDROBOT Co., LTD
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
#ifndef __LDLIDAR_DRIVER_SDK_INTERFACE_H__
#define __LDLIDAR_DRIVER_SDK_INTERFACE_H__

#include <chrono>
#include <functional>

#include "ldlidar_driver/ldlidar_datatype.h"

namespace ldlidar {

class LDLidarDriver {
public:
  LDLidarDriver();

  virtual ~LDLidarDriver();

  std::string GetLidarSdkVersionNumber(void);
  
  virtual void EnablePointCloudDataFilter(bool is_enable) = 0;

  virtual bool WaitLidarComm(int64_t timeout) = 0;

  virtual LidarStatus GetLaserScanData(Points2D& dst, int64_t timeout) = 0;

  virtual LidarStatus GetLaserScanData(LaserScan& dst, int64_t timeout) = 0;
  
  virtual bool GetLidarScanFreq(double& spin_hz) = 0;  

  virtual void RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) = 0;

  virtual uint8_t GetLidarErrorCode(void) = 0;

  /**
   * @brief Start lidar driver node
   * @param none
   * @retval value is true, start is success;
   *   value is false, start is failed.
  */
  virtual bool Start(void) = 0;
  
  /**
   * @brief Stop lidar driver node
   * @param none
   * @retval value is true, stop is success;
   *  value is false, stop is failed.
  */
  virtual bool Stop(void) = 0;

  /**
   * @brief Get SDK(ldlidar driver) running status.
  */
  static bool Ok();

  /**
   * @brief Set SDK(ldlidar driver) running status.
  */
  static void SetLidarDriverStatus(bool status);

protected:
  bool is_start_flag_;
  bool is_connect_flag_;

private:
  std::string sdk_pack_version_;
  static bool is_ok_;
};

} // namespace ldlidar

#endif // __LDLIDAR_DRIVER_NODE_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/