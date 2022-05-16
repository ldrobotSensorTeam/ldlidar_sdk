/**
 * @file lipkg.h
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
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
#ifndef __LIPKG_H
#define __LIPKG_H
#include <stdint.h>

#include <array>
#include <iostream>
#include <vector>
#include <mutex>

#include "pointdata.h"
#include "transform.h"

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
public:
  LiPkg(LDVersion ld_version);
  void CommReadCallBack(const char *byte, size_t len);
  bool GetLaserScanData(Points2D& out); // get a frame lidar data
  double GetSpeed(void); //get  Lidar spin speed (Hz)
  LidarStatus GetLidarStatus(void);

private:
  bool AnalysisOne(uint8_t byte); // parse single packet
  bool Parse(const uint8_t *data, long len); // ==
  bool AssemblePacket(); // combine stantard data into data frames and calibrate
  bool IsFrameReady(void);  // get Lidar data frame ready flag
  void ResetFrameReady(void);  // reset frame ready flag
  void SetFrameReady(void);    // set frame ready flag
  long GetErrorTimes(void); // the number of errors in parser process of lidar data frame
  uint64_t GetTime(void); // systime ns
  void AnalysisLidarIsBlocking(uint16_t lidar_speed_val);
  void AnalysisLidarIsOcclusion(Points2D& lidar_data);

private:
  const int kPointFrequence = 2300;
  LDVersion ld_version_;
  LidarStatus ld_lidarstatus_;
  bool is_frame_ready_;
  uint16_t timestamp_; // time stamp of the packet
  double speed_;
  long error_times_;
  LiDARFrameTypeDef pkg;
  Points2D lidar_frame_data_;
  Points2D frame_tmp_;
  std::mutex mutex_lock;
};

#endif  // __LIPKG_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/