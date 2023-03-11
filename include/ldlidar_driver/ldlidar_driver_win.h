/**
 * @file ldlidar_driver.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar sdk interface
 *         This code is only applicable to LDROBOT LiDAR LD14
 * products sold by Shenzhen LDROBOT Co., LTD
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
#ifndef __LDLIDAR_DRIVER_SDK_WIN_INTERFACE_H__
#define __LDLIDAR_DRIVER_SDK_WIN_INTERFACE_H__

#include "ldlidar_driver/ldlidar_driver.h"
#include "ldlidar_driver/ldlidar_dataprocess.h"
#include "ldlidar_driver/log_module.h"
#include "ldlidar_driver/serial_interface_win.h"

namespace ldlidar {

class LDLidarDriverWinInterface : public LDLidarDriver {
public:
  LDLidarDriverWinInterface();

  ~LDLidarDriverWinInterface();
  
  /**
   * @brief Communcation port open and assert initlization param
   * @param product_name
   *        ldlidar product type: ldlidar::LDType, value:
   *          - ldlidar::LDType::NOVER
   *          - ldlidar::LDType::LD_14
   *            ....
   *        - else definition in "ldlidar_driver/include/ldlidar_datatype.h"
   * @param serial_port_name
   *        serial device system path, eg: "/dev/ttyUSB0"
   * @param serial_baudrate
   *       serial communication baudrate value(> 0), unit is bit/s.
   * @retval value is true, start is success;
   *   value is false, start is failed.
  */
  bool Connect(LDType product_name, 
              std::string& serial_port_name, 
              PortParams& port_params);
  
  /**
   * @brief Communication port close
   * @retval  value is  ture, close success.
   *   value is false, close fail.
  */
  bool Disconnect(void);
  
  void EnablePointCloudDataFilter(bool is_enable) override;

  /**
   * @brief Whether the connection of the communication channel is normal after the lidar is powered on
   * @param[in]
   * *@param timeout:  Wait timeout, in milliseconds
   * @retval if times >= 1000, return false, communication connection is fail;
   *   if "times < 1000", return ture, communication connection is successful.
  */
  bool WaitLidarComm(int64_t timeout = 1000) override;

  /**
   * @brief get lidar laser scan point cloud data
   * @param [output]
   * *@param dst: type is ldlidar::Point2D, value is laser scan point cloud data
   * @param [in]
   * *@param timeout: Wait timeout, in milliseconds
   * @retval value is ldlidar::LidarStatus Enum Type, definition in "include/ldlidar_datatype.h", value is:
   *  ldlidar::LidarStatus::NORMAL   // 雷达正常
   *  lldlidar::LidarStatus::ERROR    // 雷达异常错误
   *  ....
  */
  LidarStatus GetLaserScanData(Points2D& dst, int64_t timeout = 1000) override;

  LidarStatus GetLaserScanData(LaserScan& dst, int64_t timeout = 1000) override;
  
  /**
   * @brief get lidar scan frequence
   * @param [output]
   * *@param spin_hz: value is lidar scan frequence, unit is Hz
   * @retval value is true, get sucess; 
  */
  bool GetLidarScanFreq(double& spin_hz) override;  

  /**
   * @brief register get timestamp handle functional.
   * @param [input]
   * *@param get_timestamp_handle: type is `uint64_t (*)(void)`, value is pointer get timestamp fuction.
   * @retval none
  */
  void RegisterGetTimestampFunctional(std::function<uint64_t(void)> get_timestamp_handle) override;

  /**
   * @brief When the lidar is in an error state, get the error code fed back by the lidar
   * @param none
   * @retval errcode
  */
  uint8_t GetLidarErrorCode(void) override;

  /**
   * @brief Start lidar driver node
   * @param none
   * @retval value is true, start is success;
   *   value is false, start is failed.
  */
  bool Start(void) override;
  
  /**
   * @brief Stop lidar driver node
   * @param none
   * @retval value is true, stop is success;
   *  value is false, stop is failed.
  */
  bool Stop(void) override;

  static LDLidarDriverWinInterface* Create(void) {
    LDLidarDriverWinInterface* pdrv = new LDLidarDriverWinInterface();
    return pdrv;
  }

  static void Destory(LDLidarDriverWinInterface* pdrv) {
    if (pdrv != nullptr) {
      delete pdrv;
    }
  }

private:
  // bool is_start_flag_;
  // bool is_connect_flag_;
  LdLidarDataProcess* comm_pkg_;
  SerialInterfaceWin* comm_port_handle_;
  std::function<uint64_t(void)> register_get_timestamp_handle_;
  std::chrono::steady_clock::time_point last_pubdata_times_;
};

} // namespace ldlidar

#endif // __LDLIDAR_DRIVER_NODE_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/