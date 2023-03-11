/**
 * @file lipkg.cpp
 * @author LDRobot (support@ldrobot.com)
 *         David Hu (hmd_hubei_cn@163.com)
 * @brief  LiDAR data protocol processing App
 * @version 1.0
 * @date 2023-03-12
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
#include "ldlidar_driver/ldlidar_dataprocess.h"

namespace ldlidar {

LdLidarDataProcess::LdLidarDataProcess()
  : lidar_measure_freq_(2300),
    typenumber_(LDType::NO_VER),
    lidarstatus_(LidarStatus::NORMAL),
    lidarerrorcode_(LIDAR_NO_ERROR),
    is_frame_ready_(false),
    is_noise_filter_(false),
    timestamp_(0),
    speed_(0),
    get_timestamp_(nullptr),
    is_poweron_comm_normal_(false),
    last_pkg_timestamp_(0),
    protocol_handle_(new LdLidarProtocol()) {

}

LdLidarDataProcess::~LdLidarDataProcess() {
  if (protocol_handle_ != nullptr) {
    delete protocol_handle_;
  }
}

void LdLidarDataProcess::SetProductType(LDType typenumber) {
  typenumber_ = typenumber;
  switch (typenumber) {
    case LDType::LD_14:
      lidar_measure_freq_ = 2300;
      break;
    case LDType::LD_14P:
      lidar_measure_freq_ = 4000;
      break;
    case LDType::LD_06:
    case LDType::LD_19:
      lidar_measure_freq_ = 4500;
      break;
    case LDType::STL_06P:
    case LDType::STL_26:
      lidar_measure_freq_ = 5000;
      break;
    case LDType::STL_27L:
      lidar_measure_freq_ = 21600;
      break;
    default :
      lidar_measure_freq_ = 2300;
      break;
  }
}

void LdLidarDataProcess::SetNoiseFilter(bool is_enable) {
  is_noise_filter_ = is_enable;
}

void LdLidarDataProcess::RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle) {
  get_timestamp_ = timestamp_handle;
}

bool LdLidarDataProcess::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {

    uint8_t ret = protocol_handle_->AnalysisDataPacket(data[i]);
    if (ret == GET_PKG_PCD) {
      LiDARMeasureDataType datapkg = protocol_handle_->GetPCDPacketData();
      is_poweron_comm_normal_ = true;
      speed_ = datapkg.speed;
      timestamp_ = datapkg.timestamp;
      // parse a package is success
      double diff = (datapkg.end_angle / 100 - datapkg.start_angle / 100 + 360) % 360;
      if (diff <= ((double)datapkg.speed * POINT_PER_PACK / lidar_measure_freq_ * 1.5)) {
        if (0 == last_pkg_timestamp_) {
          last_pkg_timestamp_ = get_timestamp_();
        } else {
          uint64_t current_pack_stamp = get_timestamp_();
          int pkg_point_number = POINT_PER_PACK;
          double pack_stamp_point_step =  
            static_cast<double>(current_pack_stamp - last_pkg_timestamp_) / static_cast<double>(pkg_point_number - 1);
          uint32_t diff =((uint32_t)datapkg.end_angle + 36000 - (uint32_t)datapkg.start_angle) % 36000;
          float step = diff / (POINT_PER_PACK - 1) / 100.0;
          float start = (double)datapkg.start_angle / 100.0;
          PointData data;
          for (int i = 0; i < POINT_PER_PACK; i++) {
            data.distance = datapkg.point[i].distance;
            data.angle = start + i * step;
            if (data.angle >= 360.0) {
              data.angle -= 360.0;
            }
            data.intensity = datapkg.point[i].intensity;
            data.stamp = static_cast<uint64_t>(last_pkg_timestamp_ + (pack_stamp_point_step * i));
            tmp_lidar_scan_data_vec_.push_back(PointData(data.angle, data.distance, data.intensity, data.stamp));
          }
          last_pkg_timestamp_ = current_pack_stamp; //// update last pkg timestamp
        }
      }
    }
  }

  return true;
}

bool LdLidarDataProcess::AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  if (speed_ <= 0) {
    tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.end());
    return false;
  }

  for (auto n : tmp_lidar_scan_data_vec_) {
    // wait for enough data, need enough data to show a circle
	// enough data has been obtained
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      if ((count * GetSpeed()) > (lidar_measure_freq_ * 1.4)) {
        if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
          tmp_lidar_scan_data_vec_.clear();
        } else {
          tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
        }
        return false;
      }
      data.insert(data.begin(), tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);

      switch (typenumber_) {
        case LDType::LD_14:
        case LDType::LD_14P: {
          SlTransform trans(typenumber_);
          data = trans.Transform(data); // transform raw data to stantard data  
          if (is_noise_filter_ && (typenumber_ != LDType::LD_14P)) {
            std::sort(data.begin(), data.end(), 
              [](PointData a, PointData b) { return a.angle < b.angle;});
            Slbf sb(speed_);
            tmp = sb.NearFilter(data); // filter noise point
          } else {
            tmp = data;
          }
          break;
        }
        case LDType::LD_06:
        case LDType::LD_19:
        case LDType::STL_06P:
        case LDType::STL_26:
        case LDType::STL_27L: {
          if (is_noise_filter_) {
            Tofbf tofbfLd(speed_, typenumber_);
            tmp = tofbfLd.Filter(data); // filter noise point
          } else {
            tmp = data;
          }
          break;
        }
        default : {
          tmp = data;
          break;
        }
      }

      std::sort(tmp.begin(), tmp.end(), 
        [](PointData a, PointData b) { return a.stamp < b.stamp; });
      if (tmp.size() > 0) {
        SetLaserScanData(tmp);
        SetFrameReady();

        if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
          tmp_lidar_scan_data_vec_.clear();
        } else {
          tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
        }
        return true;
      }
    }
    count++;

    if ((count * GetSpeed()) > (lidar_measure_freq_ * 2)) {
      if (count >= (int)tmp_lidar_scan_data_vec_.size()) {
        tmp_lidar_scan_data_vec_.clear();
      } else {
        tmp_lidar_scan_data_vec_.erase(tmp_lidar_scan_data_vec_.begin(), tmp_lidar_scan_data_vec_.begin() + count);
      }
      return false;
    }

    last_angle = n.angle;
  }

  return false;
}

void LdLidarDataProcess::CommReadCallback(const char *byte, size_t len) {
  if (Parse((uint8_t *)byte, len)) {
    AssemblePacket();
  }
}

bool LdLidarDataProcess::GetLaserScanData(Points2D& out) {
  if (IsFrameReady()) {
    ResetFrameReady();
    out = GetLaserScanData();
    return true;
  } else {
    return false;
  }
}

double LdLidarDataProcess::GetSpeed(void) { 
  return (speed_ / 360.0);  // unit  is Hz
}

LidarStatus LdLidarDataProcess::GetLidarStatus(void) {
  return lidarstatus_;
}

uint8_t LdLidarDataProcess::GetLidarErrorCode(void) {
  return lidarerrorcode_;
}

bool LdLidarDataProcess::GetLidarPowerOnCommStatus(void) {
  if (is_poweron_comm_normal_) {
    is_poweron_comm_normal_ = false;
    return true;
  } else {
    return false;
  }
}

void LdLidarDataProcess::SetLidarStatus(LidarStatus status) {
  lidarstatus_ = status;
}

void LdLidarDataProcess::SetLidarErrorCode(uint8_t errorcode) {
  lidarerrorcode_ = errorcode;
}

bool LdLidarDataProcess::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return is_frame_ready_; 
}

void LdLidarDataProcess::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = false;
}

void LdLidarDataProcess::SetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = true;
}

void LdLidarDataProcess::SetLaserScanData(Points2D& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  lidar_scan_data_vec_ = src;
}

Points2D LdLidarDataProcess::GetLaserScanData(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  return lidar_scan_data_vec_; 
}

}  // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/