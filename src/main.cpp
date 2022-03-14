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

#include <stdio.h>
#include <unistd.h>

#include <iostream>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  std::cout << "[ldrobot] SDK Pack Version is " << "v2.1.4" << std::endl;
  
  if (argc != 3) {
    std::cout << "[ldrobot] please input: ./ldlidar_sl <product_name> <serial_number>" << std::endl;
    std::cout << "[ldrobot] example:" << std::endl;
    std::cout << "./ldlidar_sl LD** /dev/ttyUSB*" << std::endl;
    std::cout << "or" << std::endl;
    std::cout << "./ldlidar_sl LD** /dev/ttyS*" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string product(argv[1]);
  
  LiPkg *lidar = nullptr;
  uint32_t baudrate = 0;
  if (product == "LD14") {
    baudrate = 115200;
    lidar = new LiPkg(LDVersion::LD_FOURTEEN);
  } else {
    perror("[ldrobot] lidar product_name is not correct !!!\n");
    exit(EXIT_FAILURE);
  }

  CmdInterfaceLinux cmd_port(baudrate);
  std::string port_name(argv[2]);
  
  if (port_name.empty()) {
    std::cout << "[ldrobot] Can't find LiDAR_" << product << std::endl;
    exit(EXIT_FAILURE);
  }else {
	  std::cout << "[ldrobot] Found LiDAR_" << product << " " << port_name << std::endl;
  }

  cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
    if (lidar->Parse((uint8_t *)byte, len)) {
      lidar->AssemblePacket();
    }
  });

  if (!cmd_port.Open(port_name)) {
    perror("[ldrobot] serial is not open !!!\n");
    exit(EXIT_FAILURE);
  }

  Points2D laser_scan;

  while (1) {
    if (lidar->IsFrameReady()) {
      std::cout << "[ldrobot] speed(Hz)： " << lidar->GetSpeed() << std::endl;
      std::cout << "[ldrobot] pack errcount: " << lidar->GetErrorTimes() << std::endl;
      laser_scan = lidar->GetData();
      std::cout << "[ldrobot] laser_scan.size() " << laser_scan.size() << std::endl;
      for (auto ele : laser_scan) {
        std::cout << "[ldrobot] angle: " << ele.angle << " "
                  << "distance(mm): " << ele.distance << " "
                  << "intensity: " << (int)ele.intensity << " " << std::endl;
      }
      lidar->ResetFrameReady();
    }
    // usleep(1000);  // sleep 1ms
  }

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
