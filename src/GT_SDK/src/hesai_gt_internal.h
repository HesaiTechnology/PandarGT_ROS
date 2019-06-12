/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef SRC_HESAI_GT_INTERNAL_H_
#define SRC_HESAI_GT_INTERNAL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>

#include <boost/function.hpp>

#include "hesai_gt/point_types.h"
#include "src/input.h"

#define SOB_SIZE          (2)
#define LASER_NUM_SIZE    (1)
#define UNIT_NUM_SIZE     (1)
#define RT_START_SIZE     (1)
#define DIS_UNIT_SIZE     (1)
#define VERTICAL_FOV_SIZE (2)
#define SHOW_ANGLE_OFFSET (2)
#define RESERVED_SIZE     (6)
#define HEADER_SIZE       (SOB_SIZE + LASER_NUM_SIZE + UNIT_NUM_SIZE + \
    RT_START_SIZE + DIS_UNIT_SIZE + VERTICAL_FOV_SIZE + SHOW_ANGLE_OFFSET + \
    RESERVED_SIZE)
#define CHANNEL_UNIT_SIZE (7)
#define UTC_TIME_SIZE     (8)
#define FINE_TIME_SIZE    (8)
#define TEMP_NUM          (5)
#define TEMP_SIZE         (2)
#define TAIL_SIZE         (UTC_TIME_SIZE + FINE_TIME_SIZE + TEMP_NUM * \
    TEMP_SIZE)
#define FRAME_COUNT       (1000)

struct HesaiGTUnit_s {
  float    horizon;
  float    vertical;
  uint16_t distance;
  uint8_t  reflectivity;
};

typedef struct HesaiGTUnit_s HesaiGTUnit;

struct HesaiGTPacket_t {
  HesaiGTUnit_s *unitSet;
  uint8_t       laserNum;
  uint8_t       unitNum;
  uint8_t       unitSize;
  uint8_t       disUnit;
  struct tm     t;
  uint64_t      usec;
};

typedef struct HesaiGTPacket_t HesaiGTPacket;

struct ChannelCoef_t {
  float coefX;
  float coefY;
  float coefZ;
};

typedef struct ChannelCoef_t ChannelCoef; 

#define MAX_CHANNEL_COUNT  (5)

class HesaiGT_Internal {
 public:
  /**
   * @brief Constructor
   * @param device_ip         The ip of the device
   *        lidar_port        The port number of lidar data
   *        pcl_callback      The callback of PCL data structure
   *        tz                The offset of timestamp
   *        frame_id          The frame id in pcl data
   */
  HesaiGT_Internal(
      std::string device_ip, uint16_t lidar_port, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, int tz, std::string frame_id);
  /**
   * @brief Constructor
   * @param pcap_path         The pcap path
   *        pcl_callback      The callback of PCL data structure
   *        tz                The offset of timestamp
   *        frame_id          The frame id in pcl data
   */
  HesaiGT_Internal(
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, int tz, std::string frame_id);
  ~HesaiGT_Internal();

  /**
   * @brief load the correction file
   * @param correction The path of correction file
   */
  int  LoadCorrectionFile(std::string correction);
  int  Start();
  void Stop();

 private:
  void RecvTask();
  void ProcessLiarPacket();
  void PushLiDARData(DataPacket packet);
  int  ParseRawData(HesaiGTPacket *packet, const uint8_t *buf, const int len);
  void CalcPointXYZIT(HesaiGTPacket *pkt, int unitId, int channelId, \
      boost::shared_ptr<PPointCloud> cld);
  void findMaxAndMinAngle(float& max, float& min, HesaiGTPacket pkt);
  void generateCoef(int channelID, float gama, float theta);

  bool                           read_from_pcap_;
  pthread_mutex_t                lidar_lock_;
  sem_t                          lidar_sem_;
  boost::thread                  *lidar_recv_thr_;
  boost::thread                  *lidar_process_thr_;
  bool                           enable_lidar_recv_thr_;
  bool                           enable_lidar_process_thr_;
  std::list<struct DataPacket_s> lidar_packets_;
  boost::shared_ptr<Input>       input_;
  uint16_t                       last_vertical_;
  int                            tz_second_;
  std::string                    frame_id_;
  float                          max_angle_;
  float                          min_angle_;
  float                          gama_angle_[MAX_CHANNEL_COUNT];
  float                          theta_angle_[MAX_CHANNEL_COUNT];
  ChannelCoef                    coef_channels_[MAX_CHANNEL_COUNT];
  float                          cos_install_;
  float                          sin_install_;

  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> \
      pcl_callback_;
};

#endif  // SRC_HESAI_GT_INTERNAL_H_

