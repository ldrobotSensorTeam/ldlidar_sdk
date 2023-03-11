/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 1.0
 * @date 2023-2-20
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
#include "ldlidar_driver/ldlidar_driver_win.h"

uint64_t GetTimestamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

int main(int argc, char** argv) {
  struct lidar_inf_struct {
    uint8_t index;
    std::string type_name;
    ldlidar::LDType type_number;
    uint32_t baudrate;
  };

  const int inf_array_size = 4;
  lidar_inf_struct inf_array[inf_array_size] = {
      {0, "LDLiDAR LD06", ldlidar::LDType::LD_06, 230400},
      {1, "LDLiDAR LD14", ldlidar::LDType::LD_14, 115200},
      {2, "LDLiDAR LD14P", ldlidar::LDType::LD_14P, 230400},
      {3, "LDLiDAR LD19", ldlidar::LDType::LD_19, 230400}};

  printf("The SDK support ldlidar product:\n");
  for (auto inf_base : inf_array) {
    printf("index:[%d] --- %s\n", inf_base.index, inf_base.type_name.c_str());
  }
  uint8_t product_index;
  printf("\nplease select product and input index value:\n");
  scanf_s("%d", &product_index);
  if ((product_index < 0) || (product_index >= inf_array_size)) {
    return -1;
  }

  PortParams port_param;
  port_param.baudrate = (BaudRate)inf_array[product_index].baudrate;
  ldlidar::LDType lidar_type_number = inf_array[product_index].type_number;

  std::vector<PortInfo> info;
  SerialInterfaceWin::availablePorts(info);  // get serial port device info
  printf("\nAvailable port:\n\n");
  int port_index = 0;
  for (auto port_item : info) {
    printf("index:[%d] %s,    %s,\n", 
      port_index++, port_item.name.c_str(), 
      port_item.description.c_str());
  }

  printf("\nplease select port and input index value:\n");
  scanf_s("%d", &port_index);
  if (port_index > info.size()) {
    return -1;
  }

  ldlidar::LDLidarDriverWinInterface* ldlidar_drv = 
    ldlidar::LDLidarDriverWinInterface::Create();

  LOG_INFO_LITE("LDLiDAR SDK Pack Version is %s",
                ldlidar_drv->GetLidarSdkVersionNumber().c_str());

  ldlidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp));

  ldlidar_drv->EnablePointCloudDataFilter(true);

  if (ldlidar_drv->Connect(lidar_type_number, info[port_index].name, port_param)) {
    LOG_INFO_LITE("ldlidar serial connect is success", "");
  } else {
    LOG_ERROR_LITE("ldlidar serial connect is fail", "");
    exit(EXIT_FAILURE);
  }

  if (ldlidar_drv->WaitLidarComm(3500)) {
    LOG_INFO_LITE("ldlidar communication is normal.", "");
  } else {
    LOG_ERROR_LITE("ldlidar communication is abnormal.", "");
    ldlidar_drv->Disconnect();
  }

  if (ldlidar_drv->Start()) {
    LOG_INFO_LITE("ldlidar driver start is success.","");
  } else {
    LOG_ERROR_LITE("ldlidar driver start is fail.","");
  }

  ldlidar::Points2D laser_scan_points;
  while (ldlidar::LDLidarDriverWinInterface::Ok()) {
    switch (ldlidar_drv->GetLaserScanData(laser_scan_points, 1500)) {
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_scan_freq = 0;
        ldlidar_drv->GetLidarScanFreq(lidar_scan_freq);
#ifdef _WIN64
        LOG_INFO_LITE("speed(Hz):%f, size:%d,stamp_begin:%lu, stamp_end:%lu",
                      lidar_scan_freq, laser_scan_points.size(),
                      laser_scan_points.front().stamp,
                      laser_scan_points.back().stamp);
#else
        LOG_INFO_LITE("speed(Hz):%f, size:%d,stamp_begin:%llu, stamp_end:%llu",
                      lidar_scan_freq, laser_scan_points.size(),
                      laser_scan_points.front().stamp,
                      laser_scan_points.back().stamp);
#endif
        //  output 2d point cloud data
#if 0
        for (auto point : laser_scan_points) {
#ifdef _WIN64
          LOG_INFO_LITE("stamp(ns):%lu,angle:%f,distance(mm):%d,intensity:%d",
                        point.stamp, point.angle, point.distance,
                        point.intensity);
#else
          LOG_INFO_LITE("stamp(ns):%llu,angle:%f,distance(mm):%d,intensity:%d",
                        point.stamp, point.angle, point.distance,
                        point.intensity);
#endif
        }
#endif
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        LOG_ERROR_LITE("point cloud data publish time out, please check your lidar device.","");
        ldlidar_drv->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }

    Sleep(166);  // sleep 166ms , 6hz
  }

  ldlidar_drv->Stop();
  ldlidar_drv->Disconnect();

  ldlidar::LDLidarDriverWinInterface::Destory(ldlidar_drv);

  system("pause");
  return 0;
}