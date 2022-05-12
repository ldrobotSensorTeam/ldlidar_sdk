/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
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

#include "ldlidar_node.h"

int main(int argc, char **argv) {
  
  if (argc != 3) {
    perror("[ldrobot] cmd error");
    std::cout << "[ldrobot] please input: ./ldlidar_sl <product_name> <serial_number>" << std::endl;
    std::cout << "[ldrobot] example:" << std::endl;
    std::cout << "./ldlidar_sl LD** /dev/ttyUSB*" << std::endl;
    std::cout << "or" << std::endl;
    std::cout << "./ldlidar_sl LD** /dev/ttyS*" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string product(argv[1]);
  std::string port_name(argv[2]);
  ldlidar::LDlidarNode* node = nullptr;

  if (product == "LD14") {
    node = new ldlidar::LDlidarNode(LDVersion::LD_14,port_name);
  } else {
    perror("[ldrobot] lidar product_name is not correct !!!\n");
    exit(EXIT_FAILURE);
  }

  if (node->StartNode()) {
    std::cout << "[ldrobot] ldldiar node start is success" << std::endl;
  } else {
    fprintf(stderr, "[ldrobot] ERROR: lidar node start is failed\n");
    exit(EXIT_FAILURE);
  }

  Points2D laser_scan;
  double lidar_spin_freq;
  LidarStatus lidar_status;

  while (1) {
    if (node->GetLidarWorkStatus(lidar_status)) {
      switch (lidar_status){
        case LidarStatus::NORMAL:
          if (node->GetLaserScan(laser_scan)) {
            // get lidar normal data
            if (node->GetLidarSpinFreq(lidar_spin_freq)) {
              std::cout << "[ldrobot] speed(Hz): " << lidar_spin_freq << std::endl;
            }
            for (auto ele : laser_scan) {
              std::cout << "[ldrobot] angle: " << ele.angle << " "
                        << "distance(mm): " << ele.distance << " "
                        << "intensity: " << (int)ele.intensity << " "
                        << "stamp(ns): " << ele.stamp << std::endl;
            }
          }
          break;
        case LidarStatus::BLOCKING:
          std::cout << "lidar status is blocking" << std::endl;
          break;
        case LidarStatus::OCCLUSION:
          std::cout << "lidar status is occlusion" << std::endl;
          break;
        default:
          break;
      }
    }
    // usleep(1000);  // sleep 1ms
  }

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
