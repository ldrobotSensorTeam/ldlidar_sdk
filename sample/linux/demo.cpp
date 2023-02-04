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

// void LidarPowerOn(void) {
//   LOG_DEBUG("Lidar Power On","");
//   // ...
// }

// void LidarPowerOff(void) {
//   LOG_DEBUG("Lidar Power Off","");
//   // ...
// }

int main(int argc, char **argv) {
  
  if (argc != 2) {
    LOG_INFO("cmd error","");
    LOG_INFO("please input: ./ldlidar <serial_number>","");
    LOG_INFO("example:","");
    LOG_INFO("./ldlidar /dev/ttyUSB0","");
    LOG_INFO("or","");
    LOG_INFO("./ldlidar /dev/ttyS0","");
    exit(EXIT_FAILURE);
  }
  
  std::string port_name(argv[1]);
  
  ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();
  
  LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

  node->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 

  node->EnableFilterAlgorithnmProcess(true);

  if (node->Start(ldlidar::LDType::LD_14, port_name)) {
    LOG_INFO("ldlidar node start is success","");
    // LidarPowerOn();
  } else {
    LOG_ERROR("ldlidar node start is fail","");
    exit(EXIT_FAILURE);
  }

  if (node->WaitLidarCommConnect(3500)) {
    LOG_INFO("ldlidar communication is normal.","");
  } else {
    LOG_ERROR("ldlidar communication is abnormal.","");
    node->Stop();
  }
  
  ldlidar::Points2D laser_scan_points;
  while (ldlidar::LDLidarDriver::IsOk()) {

    switch (node->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_scan_freq = 0;
        node->GetLidarScanFreq(lidar_scan_freq);
#ifdef __LP64__
        LOG_INFO("speed(Hz):%f, size:%d,stamp_front:%lu, stamp_back:%lu",
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#else
        LOG_INFO("speed(Hz):%f, size:%d,stamp_front:%llu, stamp_back:%llu",
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#endif
        //  output 2d point cloud data
        for (auto point : laser_scan_points) {
#ifdef __LP64__
          LOG_INFO("stamp(ns):%lu,angle:%f,distance(mm):%d,intensity:%d", 
              point.stamp, point.angle, point.distance, point.intensity);
#else
          LOG_INFO("stamp(ns):%llu,angle:%f,distance(mm):%d,intensity:%d", 
              point.stamp, point.angle, point.distance, point.intensity);
#endif
        }
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        LOG_ERROR("point cloud data publish time out, please check your lidar device.","");
        node->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }

    usleep(1000*166);  // sleep 166ms , 6hz
  }

  node->Stop();
  // LidarPowerOff();

  delete node;
  node = nullptr;

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
