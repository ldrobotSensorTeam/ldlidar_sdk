/**
 * @file ldlidar_node.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  ldlidar processing App
 *         This code is only applicable to LDROBOT LiDAR LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-05-12
 *
 * @copyright Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights
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
#ifndef __LDLIDAR_NODE_H__
#define __LDLIDAR_NODE_H__

#include <stdio.h>
#include <unistd.h>

#include <iostream>

#include "cmd_interface_linux.h"
#include "lipkg.h"

namespace ldlidar {

class LDlidarNode {
  private:
  LDVersion product_name_;
  std::string serial_port_name_;
  LiPkg* comm_pkg_;
  CmdInterfaceLinux* comm_serial_;
  bool is_init_flag_;
  bool is_start_flag;

  public:
  LDlidarNode(LDVersion product_name, std::string serial_port_name);
  ~LDlidarNode();
  bool StartNode(void);
  bool StopNode(void);
  bool GetLaserScan(Points2D& output); 
  bool GetLidarSpinFreq(double& spin_hz);
  bool GetLidarWorkStatus(LidarStatus& status);
};

}
#endif // __LDLIDAR_NODE_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/