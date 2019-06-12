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

#include "hesai_gt/hesai_gt.h"

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  printf("lidar: pcl size: %d timestamp: %lf\n", cld->width, timestamp);
}

int main(int argc, char** argv) {
  HesaiGT hesaiGT(std::string("192.168.20.51"), 8080, lidarCallback, \
      0, std::string("hesai_gt"));
  hesaiGT.Start();

  while (true) {
    sleep(100);
  }
}

