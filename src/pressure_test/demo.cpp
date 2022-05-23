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

int pressure_test(int argc, char **argv) {
  
  // if (argc != 2) {
  //   std::cerr << "[ldrobot] cmd error" << std::endl;
  //   std::cerr << "[ldrobot] please input: ./ldlidar_sl <serial_number>" << std::endl;
  //   std::cerr << "[ldrobot] example:" << std::endl;
  //   std::cerr << "./ldlidar_sl /dev/ttyUSB*" << std::endl;
  //   std::cerr << "or" << std::endl;
  //   std::cerr << "./ldlidar_sl /dev/ttyS*" << std::endl;
  //   exit(EXIT_FAILURE);
  // }
  
  std::string port_name("/dev/ttyUSB0");
  
  ldlidar::LDlidarNode* node = new ldlidar::LDlidarNode();
  
  std::cout << "[ldrobot] SDK Pack Version is " << node->GetSdkVersionNum() << std::endl;
  if (node->StartNode(ldlidar::LDType::LD_14, port_name, true, true)) {
    std::cout << "[ldrobot] ldldiar node start is success" << std::endl;
  } else {
    std::cerr << "[ldrobot] ERROR: lidar node start is fail" << std::endl;
    exit(EXIT_FAILURE);
  }

  ldlidar::Points2D laser_scan;
  double lidar_spin_freq;
  ldlidar::LidarStatus lidar_status;
  auto last_time = std::chrono::steady_clock::now();
  int cnt = 100;
  while (cnt--) {
    if (node->GetLidarWorkStatus(lidar_status)) {
      switch (lidar_status){
        case ldlidar::LidarStatus::NORMAL:
          if (node->GetLaserScan(laser_scan)) {
            last_time = std::chrono::steady_clock::now();
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
        case ldlidar::LidarStatus::BLOCKING:
          std::cout << "lidar status is blocking" << std::endl;
          break;
        case ldlidar::LidarStatus::OCCLUSION:
          std::cout << "lidar status is occlusion" << std::endl;
          break;
        default:
          break;
      }
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) {
			std::cout << "[ldrobot] lidar pub data is time out, please check lidar device" << std::endl;
			exit(EXIT_FAILURE);
		}
    usleep(1000*166);  // sleep 166ms , 6hz
  }

  node->StopNode();

  delete node;
  node = nullptr;

  return 0;
}

int main(int argc, char **argv) {
  
  for (int i = 0; i < 10000; i++)
  {
    pressure_test(argc, argv);
  }

  while (1) {
    sleep(1);
  }
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
