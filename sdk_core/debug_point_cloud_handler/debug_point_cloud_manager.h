//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef DEBUG_POINT_CLOUD_MANAGER_H_
#define DEBUG_POINT_CLOUD_MANAGER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "debug_point_cloud_handler.h"

#include "base/io_thread.h"
#include "base/logging.h"
#include "comm/define.h"
#include "base/network/network_util.h"

namespace livox {
namespace lidar {

class DebugPointCloudManager {
 public:
  DebugPointCloudManager(const DebugPointCloudManager& other) = delete;
  DebugPointCloudManager& operator=(const DebugPointCloudManager& other) = delete;
  ~DebugPointCloudManager();

  void AddDevice(const uint32_t handle, const DetectionData* detection_data);
  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);
  bool Enable(bool enable);
  bool SetStorePath(std::string path);
  
  static DebugPointCloudManager& GetInstance();

 private:
  DebugPointCloudManager();

 private:
  std::atomic<bool> enable_{false};
  std::string path_;
  std::map<uint32_t, LidarDeviceInfo> devices_info_;
  std::map<uint32_t, std::shared_ptr<DebugPointCloudHandler>> handlers_;
};

} // namespace lidar
}  // namespace livox

#endif  // DEBUG_POINT_CLOUD_MANAGER_H_
