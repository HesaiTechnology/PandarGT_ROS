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

#ifndef INCLUDE_HESAI_GT_H_
#define INCLUDE_HESAI_GT_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>

#include <boost/function.hpp>

#include "hesai_gt/point_types.h"

class HesaiGT_Internal;

class HesaiGT {
 public:
  /**
   * @brief Constructor
   * @param device_ip         The ip of the device
   *        lidar_port        The port number of lidar data
   *        pcl_callback      The callback of PCL data structure
   *        tz                The offset of timestamp
   *        frame_id          The frame id in pcl data
   */
  HesaiGT(std::string device_ip, uint16_t lidar_port, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, int tz, std::string frame_id);

  /**
   * @brief Constructor
   * @param pcap_path         The pcap path
   *        pcl_callback      The callback of PCL data structure
   *        tz                The offset of timestamp
   *        frame_id          The frame id in pcl data
   */
  HesaiGT(std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, int tz, std::string frame_id);

  /**
   * @brief deconstructor
   */
  ~HesaiGT();

  /**
   * @brief load the lidar correction file
   * @param contents The correction contents of lidar correction
   */
  int LoadCorrectionFile(std::string contents);

  /**
   * @brief Run SDK.
   */
  int Start();

  /**
   * @brief Stop SDK.
   */
  void Stop();

 private:
  HesaiGT_Internal *internal_;
};

#endif  // INCLUDE_HESAI_GT_H_

