/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-08
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

#include "ldlidar_driver.h"

uint64_t GetTimestamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

int main(int argc, char **argv) {
  
  if (argc != 2) {
    LDS_LOG_INFO("cmd error","");
    LDS_LOG_INFO("please input: ./ldlidar_sl_node <serial_number>","");
    LDS_LOG_INFO("example:","");
    LDS_LOG_INFO("./ldlidar_sl_node /dev/ttyUSB0","");
    LDS_LOG_INFO("or","");
    LDS_LOG_INFO("./ldlidar_sl_node /dev/ttyS0","");
    exit(EXIT_FAILURE);
    exit(EXIT_FAILURE);
  }
  
  std::string port_name(argv[1]);
  
  ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();
  
  LDS_LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

  node->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 

  node->EnableFilterAlgorithnmProcess(true);

  if (node->Start(ldlidar::LDType::LD_14, port_name)) {
    LDS_LOG_INFO("ldldiar node start is success","");
  } else {
    LDS_LOG_ERROR("ldlidar node start is fail","");
    exit(EXIT_FAILURE);
  }

  if (node->WaitLidarCommConnect(3500)) {
    LDS_LOG_INFO("ldlidar communication is normal.","");
  } else {
    LDS_LOG_ERROR("ldlidar communication is abnormal.","");
    exit(EXIT_FAILURE);
  }
  
  ldlidar::Points2D laser_scan_points;
  while (ldlidar::LDLidarDriver::IsOk()) {

    switch (node->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_spin_freq = 0;
        node->GetLidarSpinFreq(lidar_spin_freq);

        LDS_LOG_INFO("speed(Hz):%f, size:%d,stamp_front:%lu, stamp_back:%lu",
            lidar_spin_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
        
        for (auto point : laser_scan_points) {
          LDS_LOG_INFO("stamp(ns):%lu,angle:%f,distance(mm):%d,intensity:%d", 
              point.stamp, point.angle, point.distance, point.intensity);
        }
        break;
      }
      case ldlidar::LidarStatus::ERROR: {
        uint8_t errcode = node->GetLidarErrorCode();
        LDS_LOG_ERROR("ldlidar feedback errcode:%d",errcode);
        if (LIDAR_ERROR_BLOCKING == errcode) {
          LDS_LOG_WARN("ldlidar blocking","");
        } else if (LIDAR_ERROR_OCCLUSION == errcode) {
          LDS_LOG_WARN("ldlidar occlusion","");
        } else if (LIDAR_ERROR_BLOCKING_AND_OCCLUSION == errcode) {
          LDS_LOG_WARN("ldlidar blocking and occlusion","");
        }
        node->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        LDS_LOG_ERROR("ldlidar point cloud data publish time out, please check your lidar device.","");
        node->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      case ldlidar::LidarStatus::STOP: {
        break;
      }
      default:
        break;
    }

    usleep(1000*166);  // sleep 166ms , 6hz
  }

  node->Stop();

  delete node;
  node = nullptr;

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
