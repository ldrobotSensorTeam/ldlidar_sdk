/**
* @file         ldlidar_protocol.h
* @author       LDRobot (support@ldrobot.com)
*               David Hu(hmd_hubei_cn@163.com)
* @brief         
* @version      1.0
* @date         2023.3.11
* @note          
* @copyright    Copyright (c) 2017-2023  SHENZHEN LDROBOT CO., LTD. All rights reserved.
* Licensed under the MIT License (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License in the file LICENSE
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**/

#ifndef __LDLIDAR_PROTOCOL_H__
#define __LDLIDAR_PROTOCOL_H__

#include <stdint.h>
#include <string.h>

namespace ldlidar {

struct CmdIdAckFlag {
  bool is_ack_start;
  bool is_ack_stop;
  bool is_ack_set_scan_freq;
  bool is_ack_get_motor_board_ver;

  CmdIdAckFlag() {
    is_ack_start = false;
    is_ack_stop = false;
    is_ack_set_scan_freq = false;
    is_ack_get_motor_board_ver = false;
  }

  void Clear() {
    is_ack_start = false;
    is_ack_stop = false;
    is_ack_set_scan_freq = false;
    is_ack_get_motor_board_ver = false;
  }
};

#define CMD_ACK_PROTOCOL_DATABUFF_MAX_SIZE   50
#define CMD_ACK_PROTOCOL_DATABUFF_MIN_SIZE   4
#define CMD_ACK_PROTOCOL_DATABUFF_INDEX      3
#define CMD_ACK_PROTOCOL_NO_DATABUFF_SIZE    4

typedef enum LidarCmdModeEnum {
  SET_START_LIDAR_SCAN = 0xA0,
  SET_STOP_LIDAR_SCAN = 0xA1,
  SET_LIDAR_SCAN_FRE = 0xA2,
  GET_LIDAR_MOTOR_BOARD_FIRMWARE = 0xA8
} LidarCmdIDType;

struct LidarCmdAckData {
  LidarCmdIDType cmd_mode;
  uint8_t databuff[CMD_ACK_PROTOCOL_DATABUFF_MAX_SIZE];
};

#ifdef __linux__

typedef struct __attribute__((packed)) {
  uint8_t  header;
  uint8_t  mode;
  uint8_t datalen;
  uint32_t data;
  uint8_t  crc8;
} LidarCmdDataType;

#else

#pragma pack(1)
typedef struct {
  uint8_t  header;
  uint8_t  mode;
  uint8_t datalen;
  uint32_t data;
  uint8_t  crc8;
} LidarCmdDataType;
#pragma pack()

#endif

// ----

#define  PKG_HEADER        0x54
#define  DATA_PKG_INFO     0x2C
#define  POINT_PER_PACK    12
#define  HEALTH_PKG_INFO   0xE0
#define  MANUFACT_PKG_INF  0x0F

#define  GET_PKG_PCD      1
#define  GET_PKG_HEALTH   2
#define  GET_PKG_MANUFACT 3
#define  GET_PKG_ERROR    0

#ifdef __linux__

typedef struct  __attribute__((packed)) {
  uint8_t header; 
  uint8_t information; 
  uint16_t speed;
  uint16_t product_version;   
  uint32_t sn_high;
  uint32_t sn_low;
  uint32_t hardware_version;
  uint32_t firmware_version;
  uint8_t crc8;
} LiDARManufactureInfoType;

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructType point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARMeasureDataType;

typedef struct __attribute__((packed)) {
  uint8_t  header;
  uint8_t  information;
  uint8_t error_code;
  uint8_t  crc8;
} LiDARHealthInfoType;

#else

#pragma pack(1)
typedef struct {
  uint8_t header; 
  uint8_t information; 
  uint16_t speed;
  uint16_t product_version;   
  uint32_t sn_high;
  uint32_t sn_low;
  uint32_t hardware_version;
  uint32_t firmware_version;
  uint8_t crc8;
} LiDARManufactureInfoType;

typedef struct {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructType;

typedef struct {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructType point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARMeasureDataType;

typedef struct {
  uint8_t  header;
  uint8_t  information;
  uint8_t error_code;
  uint8_t  crc8;
} LiDARHealthInfoType;
#pragma pack()

#endif

class LdLidarProtocol {
public:
  LdLidarProtocol();
  ~LdLidarProtocol();

  /**
   * @brief analysis data packet. . 
   * @param[in]
   * * @param   byte :  input serial byte data
   * @retval
   *   If the return value is GET_PKG_PCD macro, the lidar point cloud data is obtained
   *   If the return value is GET_PKG_HEALTH macro, the lidar health information is obtained.
   *   If the return value is GET_PKG_MANUFACT macro, the lidar manufacture information is obtained.
  */
  uint8_t AnalysisDataPacket(uint8_t byte); 
  /**
   * @brief get point cloud data.
  */
  LiDARMeasureDataType& GetPCDPacketData(void);
  /**
   * @brief get health information
  */
  LiDARHealthInfoType& GetHealthPacketData(void);

  /**
   * @brief get manufacture information
  */
  LiDARManufactureInfoType& GetManufactureInfoPacketData(void);

private:
  LiDARMeasureDataType pcdpkg_data_;
  LiDARHealthInfoType healthpkg_data_;
  LiDARManufactureInfoType manufacinfpkg_data_;
};

#ifdef __cplusplus
extern "C" {
#endif

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif


} // namespace ldlidar

#endif //__LDLIDAR_PROTOCOL_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
