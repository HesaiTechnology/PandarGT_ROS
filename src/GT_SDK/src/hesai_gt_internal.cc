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

#include <math.h>
#include <sstream>
#include "src/input.h"
#include "src/hesai_gt_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_DISCONNET_COUNT       (60)
#define ANGLE_TO_RADIAN(x)        ((x) * M_PI / 180.0)
#define GT_VERSION3_CHANNEL_COUNT (3)
#define METER_TO_MILLIMETER       (1000.0f)
#define INSTALL_ANGLE             (-20.0f)
#define SEC_TO_USEC               (1000000.0)
#define PCAP_TIME_INTERVAL        (100000 / 3000)

double degreeToRadian(double degree) { return degree * M_PI / 180; }

HesaiGT_Internal::HesaiGT_Internal(
    std::string device_ip, uint16_t lidar_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback, \
    int tz, std::string frame_id) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_    = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_    = false;
  enable_lidar_process_thr_ = false;

  input_.reset(new Input(lidar_port, lidar_port));

  read_from_pcap_ = false;
  pcl_callback_   = pcl_callback;
  frame_id_       = frame_id;
  tz_second_      = tz * 3600;
  cos_install_    = cosf(ANGLE_TO_RADIAN(INSTALL_ANGLE));
  sin_install_    = sinf(ANGLE_TO_RADIAN(INSTALL_ANGLE));
}

HesaiGT_Internal::HesaiGT_Internal(
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, int tz, std::string frame_id) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_    = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_    = false;
  enable_lidar_process_thr_ = false;

  input_.reset(new Input(pcap_path));

  read_from_pcap_ = true;
  pcl_callback_   = pcl_callback;
  frame_id_       = frame_id;
  tz_second_      = tz * 3600;
  cos_install_    = cosf(ANGLE_TO_RADIAN(INSTALL_ANGLE));
  sin_install_    = sinf(ANGLE_TO_RADIAN(INSTALL_ANGLE));
}

HesaiGT_Internal::~HesaiGT_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int HesaiGT_Internal::LoadCorrectionFile(std::string correction_content) {
  std::ifstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    if (lineCounter++ >= MAX_CHANNEL_COUNT) break;

    int   lineId;
    float gama;
    float theta;

    std::cout << line << std::endl;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> gama;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> theta;

    if (lineId != lineCounter) {
      return -1;
    }

    gama_angle_[lineId-1]  = gama;
    theta_angle_[lineId-1] = theta;

    generateCoef(lineId - 1, gama, theta);
  }

  return 0;
}

void HesaiGT_Internal::generateCoef(int channelID, float gama, float theta) {
  float sinGama  = sinf(ANGLE_TO_RADIAN(gama));
  float cosGama  = cosf(ANGLE_TO_RADIAN(gama));
  float sinTheta = sinf(ANGLE_TO_RADIAN(theta));
  float cosTheta = cosf(ANGLE_TO_RADIAN(theta));

  coef_channels_[channelID].coefX = sinTheta;
  coef_channels_[channelID].coefY = -1 * cosTheta * cosGama;
  coef_channels_[channelID].coefZ = -1 * cosTheta * sinGama;
}

int HesaiGT_Internal::Start() {
  Stop();
  enable_lidar_recv_thr_    = true;
  enable_lidar_process_thr_ = true;

  lidar_process_thr_ = new boost::thread(\
      boost::bind(&HesaiGT_Internal::ProcessLiarPacket, this));
  lidar_recv_thr_    = new boost::thread(\
      boost::bind(&HesaiGT_Internal::RecvTask, this));
}

void HesaiGT_Internal::Stop() {
  enable_lidar_recv_thr_    = false;
  enable_lidar_process_thr_ = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }
  return;
}

void HesaiGT_Internal::RecvTask() {
  int ret = 0;

  while (enable_lidar_recv_thr_) {
    DataPacket pkt;
    int        rc = 0;

    if (read_from_pcap_) {
      rc = input_->getPacketFromPcap(&pkt);
    } else {
      rc = input_->getPacket(&pkt);
    }

    if (-1 == rc) {
      if (read_from_pcap_) {
        usleep(PCAP_TIME_INTERVAL);
      }
      continue;
    } else if (1 == rc) {
      return;
    }

    if (pkt.data[0] != 0xee || pkt.data[1] != 0xff){
      continue;
    }

    PushLiDARData(pkt);
  }
}

void HesaiGT_Internal::findMaxAndMinAngle(float& max, float& min, \
    HesaiGTPacket pkt) {
  max = pkt.unitSet[0].vertical;
  min = pkt.unitSet[0].vertical;
  for (uint8_t i = 1; i < pkt.laserNum * pkt.unitNum; i++) {
    if (pkt.unitSet[i].vertical > max) {
      max = pkt.unitSet[i].vertical;
    } else if (pkt.unitSet[i].vertical < min) {
      min = pkt.unitSet[i].vertical;
    }
  }
}

void HesaiGT_Internal::ProcessLiarPacket() {
  struct timespec ts;
  uint8_t         lastLaserNum      = 0;
  uint8_t         lastUnitNum       = 0;
  HesaiGTUnit     *unitSet          = NULL;
  double          firstPktTime      = 0.0;
  bool            recvDataFlag      = false;
  uint64_t        disconnectCount   = 0;
  bool            foundCutAngle     = false;
  float           maxInOneSec       = 0.0f;
  float           minInOneSec       = 0.0f;
  bool            reachMaxOnce      = false;
  bool            reachMaxTwice     = false;
  int             pktCount          = 0;

  boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());

  while (enable_lidar_process_thr_) {
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      std::cout << "get time error" << std::endl;
    }

    ts.tv_sec += 1;

    if (sem_timedwait(&lidar_sem_, &ts) == -1) {
      if (recvDataFlag) {
        // If disconnecting more than 1min,
        // we will find new cut frame angles next time.
        if (disconnectCount++ > MAX_DISCONNET_COUNT) {
          disconnectCount = 0;
          recvDataFlag    = false;
          foundCutAngle   = false;
          reachMaxOnce    = false;
          reachMaxTwice   = false;
          pktCount        = 0;
        }
      }

      continue;
    }

    disconnectCount = 0;

    pthread_mutex_lock(&lidar_lock_);
    DataPacket packet = lidar_packets_.front();
    lidar_packets_.pop_front();
    pthread_mutex_unlock(&lidar_lock_);
    pktCount++;

    HesaiGTPacket pkt;
    uint8_t       unitNum  = packet.data[SOB_SIZE+LASER_NUM_SIZE];
    uint8_t       laserNum = packet.data[SOB_SIZE];

    if (packet.size != HEADER_SIZE + laserNum * unitNum * \
        static_cast<uint16_t>(CHANNEL_UNIT_SIZE) + TAIL_SIZE) {
      printf("packet size mismatch HesaiGT_Internal\n");

      continue;
    }

    if (unitNum != lastUnitNum || laserNum != lastLaserNum) {
      if (unitSet != NULL) {
        free(unitSet);
        unitSet = NULL;
      }

      unitSet  = static_cast<HesaiGTUnit*>(malloc(laserNum * unitNum * \
          sizeof(HesaiGTUnit)));
      lastLaserNum = laserNum;
      lastUnitNum  = unitNum;
    }

    pkt.unitSet  = unitSet;
    pkt.laserNum = laserNum;
    pkt.unitNum  = unitNum;
    pkt.disUnit  = \
        packet.data[SOB_SIZE+LASER_NUM_SIZE+UNIT_NUM_SIZE+RT_START_SIZE];

    if (ParseRawData(&pkt, packet.data, packet.size) != 0) {
      printf("Parse raw data fail\n");
      continue;
    }

    if (!recvDataFlag) {
      firstPktTime = mktime(&pkt.t) + static_cast<double>(pkt.usec) / SEC_TO_USEC;
      recvDataFlag = true;
      findMaxAndMinAngle(maxInOneSec, minInOneSec, pkt);

      continue;
    } else {
      if (mktime(&pkt.t) + static_cast<double>(pkt.usec) /SEC_TO_USEC - firstPktTime > 1.0 || \
          pktCount >= FRAME_COUNT) {
        foundCutAngle = true;
        firstPktTime  = mktime(&pkt.t) + static_cast<double>(pkt.usec) / SEC_TO_USEC;
        pktCount = 0;
        max_angle_    = maxInOneSec - 0.2f;
        min_angle_    = minInOneSec + 0.2f;
      } else {
        float maxInPkt;
        float minInPkt;

        findMaxAndMinAngle(maxInPkt, minInPkt, pkt);

        if (maxInPkt > maxInOneSec) {
          maxInOneSec = maxInPkt;
        }

        if (minInPkt < minInOneSec) {
          minInOneSec = minInPkt;
        }

        if (!foundCutAngle) {
          continue;
        }
      }
    }

    for (int i = 0; i < unitNum; i++) {
      for (int j = 0; j < laserNum; j++) {
        float vertical    = pkt.unitSet[i*pkt.laserNum+j].vertical;
        bool  displayFlag = false;

        if (reachMaxTwice) {
          if (vertical >= min_angle_) {
            displayFlag = true;
          } else {
            reachMaxTwice = false;
            reachMaxOnce  = false;
            if (pcl_callback_ && outMsg->points.size() > 0) {
              pcl_callback_(outMsg, outMsg->points[0].timestamp);
              outMsg.reset(new PPointCloud());
            }
          }
        } else if (reachMaxOnce) {
          if (vertical > max_angle_) {
          } else if (vertical > min_angle_) {
            displayFlag   = true;
            reachMaxTwice = true;
          } else {
            reachMaxOnce = false;
          }
        } else {
          if (vertical >= max_angle_) {
            reachMaxOnce = true;
          }
        }

        if (displayFlag) {
          CalcPointXYZIT(&pkt, i, j, outMsg);
        }
      }
    }

    outMsg->header.frame_id = frame_id_;
    outMsg->height = 1;
  }

  if (unitSet != NULL) {
    free(unitSet);
    unitSet = NULL;
  }
}

void HesaiGT_Internal::PushLiDARData(DataPacket packet) {
  pthread_mutex_lock(&lidar_lock_);
  lidar_packets_.push_back(packet);
  pthread_mutex_unlock(&lidar_lock_);

  sem_post(&lidar_sem_);
}

int HesaiGT_Internal::ParseRawData(HesaiGTPacket *packet, const uint8_t *buf, \
    const int len) {
  if (NULL == buf) {
    return -1;
  }

  uint8_t       laserNum     = buf[SOB_SIZE];
  uint8_t       unitNum      = buf[SOB_SIZE+LASER_NUM_SIZE];
  const uint8_t *channelData = buf + HEADER_SIZE;
  int           index        = HEADER_SIZE;

  for (int i = 0; i < packet->laserNum * packet->unitNum; i++) {
    packet->unitSet[i].horizon      = static_cast<float>(buf[index+6] | \
        (static_cast<uint16_t>(buf[index+5]) << 8)) / 1024.0f - 32.0f;
    packet->unitSet[i].vertical     = static_cast<float>(buf[index+4] | \
        (static_cast<uint16_t>(buf[index+3]) << 8)) / 1024.0f - 32.0f;
    packet->unitSet[i].distance     = buf[index+2] | \
        (static_cast<uint16_t>(buf[index+1]) << 8);
    packet->unitSet[i].reflectivity = buf[index+0];

    index += CHANNEL_UNIT_SIZE;
  }


  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet->t.tm_year  = (buf[index+0] & 0xff) + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  packet->t.tm_mon   = (buf[index+1] & 0xff) - 1;
  packet->t.tm_mday  = (buf[index+2] & 0xff);
  packet->t.tm_hour  = (buf[index+3] & 0xff);
  packet->t.tm_min   = (buf[index+4] & 0xff);
  packet->t.tm_sec   = (buf[index+5] & 0xff);
  packet->t.tm_isdst = 0;

  index += UTC_TIME_SIZE;

  packet->usec = (buf[index] & 0xff) | \
      (buf[index+1] & 0xff) << 8 | \
      ((buf[index+2] & 0xff) << 16) | \
      ((buf[index+3] & 0xff) << 24);

  return 0;
}

void HesaiGT_Internal::CalcPointXYZIT(HesaiGTPacket *pkt, int unitId, \
    int channelId, boost::shared_ptr<PPointCloud> cld) {
  double unix_second =
      static_cast<double>(mktime(&pkt->t) + tz_second_);
  uint8_t index = unitId * pkt->laserNum + channelId;

  float distance    = pkt->unitSet[index].distance * \
      pkt->disUnit / METER_TO_MILLIMETER;

  if (distance < 0.5f) {
    return;
  }

  float horizon     = pkt->unitSet[index].horizon / 2.0f;
  float vertical    = pkt->unitSet[index].vertical / 2.0f;
  float sinHorizon  = sinf(ANGLE_TO_RADIAN(horizon));
  float cosHorizon  = cosf(ANGLE_TO_RADIAN(horizon));
  float sinVertical = sinf(ANGLE_TO_RADIAN(vertical));
  float cosVertical = cosf(ANGLE_TO_RADIAN(vertical));
  float coef        = \
      -cosVertical * sinHorizon * coef_channels_[channelId].coefX + \
      cosVertical * cosHorizon * coef_channels_[channelId].coefY + \
      sinVertical * coef_channels_[channelId].coefZ;
  float coefX       = coef_channels_[channelId].coefX + \
      2 * cosVertical * sinHorizon * coef;
  float coefY       = coef_channels_[channelId].coefY - \
      2 * cosVertical * cosHorizon * coef;
  float coefZ       = coef_channels_[channelId].coefZ - \
      2 * sinVertical * coef;

  PPoint point;
  point.x            = distance * coefX;
  point.y            = distance * (coefY * cos_install_ - \
      coefZ * sin_install_);
  point.z            = distance * (coefY * sin_install_ + \
      coefZ * cos_install_);
  point.intensity= pkt->unitSet[index].reflectivity;
  point.timestamp    = unix_second + static_cast<double>(pkt->usec) / SEC_TO_USEC;
  point.ring         = channelId;

  cld->push_back(point);
}

