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

#include "ldlidar_driver/ldlidar_driver.h"

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

ldlidar::LDType GetLdsType(std::string in_str) {
  if (!strcmp(in_str.c_str(),"LD14")) {
    return ldlidar::LDType::LD_14;
  } else if (!strcmp(in_str.c_str(),"LD06")) {
    return ldlidar::LDType::LD_06;
  } else if (!strcmp(in_str.c_str(),"LD19")) {
    return ldlidar::LDType::LD_19;
  } else {
    return ldlidar::LDType::NO_VER;
  }
}

uint32_t GetLdsSerialPortBaudrateValue(std::string in_str) {
  if (!strcmp(in_str.c_str(),"LD14")) {
    return 115200;
  } else if ((!strcmp(in_str.c_str(),"LD06")) || 
    (!strcmp(in_str.c_str(),"LD19"))) {
    return 230400;
  } else {
    return 0;
  }
}

int main(int argc, char **argv) {
  
  if (argc != 3) {
    LOG_INFO("cmd error","");
    LOG_INFO("please input: ./ldlidar <lidar type> <serial number>","");
    LOG_INFO("example:","");
    LOG_INFO("./ldlidar LD14 /dev/ttyUSB0","");
    LOG_INFO("or","");
    LOG_INFO("./ldlidar LD14 /dev/ttyS0","");
    exit(EXIT_FAILURE);
  }
  
  std::string ldlidar_type_str(argv[1]);
  std::string ldlidar_serial_port_name(argv[2]);

  // select ldrobot lidar sensor type.
  ldlidar::LDType ldlidar_type_dest;
  ldlidar_type_dest = GetLdsType(ldlidar_type_str);
  if (ldlidar_type_dest == ldlidar::LDType::NO_VER) {
    LOG_WARN("ldlidar_type_str value is not sure: %s", ldlidar_type_str.c_str());
    exit(EXIT_FAILURE);
  }

  // if use serial communications interface, as select serial baudrate paramters.
  uint32_t ldlidar_serial_baudrate_val;
  ldlidar_serial_baudrate_val = GetLdsSerialPortBaudrateValue(ldlidar_type_str);
  if (!ldlidar_serial_baudrate_val) {
    LOG_WARN("ldlidar_type_str value is not sure: %s", ldlidar_type_str.c_str());
    exit(EXIT_FAILURE);
  }
  
  ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();
  
  LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

  node->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 

  node->EnableFilterAlgorithnmProcess(true);

  if (node->Start(ldlidar_type_dest, ldlidar_serial_port_name, ldlidar_serial_baudrate_val)) {
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
        LOG_INFO_LITE("speed(Hz):%f, size:%d,stamp_begin:%lu, stamp_end:%lu",
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#else
        LOG_INFO_LITE("speed(Hz):%f, size:%d,stamp_begin:%llu, stamp_end:%llu",
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#endif
        //  output 2d point cloud data
#if 0
        for (auto point : laser_scan_points) {
#ifdef __LP64__
          LOG_INFO_LITE("stamp(ns):%lu,angle:%f,distance(mm):%d,intensity:%d", 
              point.stamp, point.angle, point.distance, point.intensity);
#else
          LOG_INFO_LITE("stamp(ns):%llu,angle:%f,distance(mm):%d,intensity:%d", 
              point.stamp, point.angle, point.distance, point.intensity);
#endif
        }
#endif
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        LOG_ERROR_LITE("point cloud data publish time out, please check your lidar device.","");
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
