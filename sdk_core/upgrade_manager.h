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

#ifndef LIVOX_UPGRADE_MANAGER_H_
#define LIVOX_UPGRADE_MANAGER_H_

#include <functional>
#include <memory>
#include <fstream>
#include <vector>
#include <string>

#include "upgrade/livox_lidar_upgrader.h"
#include "upgrade/firmware.h"

namespace livox {
namespace lidar {

class UpgradeManager {
public:
  using OnLivoxLidarUpgradeProgressCallback = 
      std::function<void(uint32_t handle, LivoxLidarUpgradeState state, void *client_data)>;

 private:
  UpgradeManager();
  UpgradeManager(const UpgradeManager& other) = delete;
  UpgradeManager& operator=(const UpgradeManager& other) = delete;
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;
  void Destory();
  ~UpgradeManager() = default;
  static UpgradeManager& GetInstance();

  // Livox lidar upgrade
  bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path);
  void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data);
  void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num);
  void CloseLivoxLidarFirmwareFile();
private:
  Firmware livox_lidar_firmware_;
  OnLivoxLidarUpgradeProgressCallback livox_lidar_info_cb_;
  void *livox_lidar_client_data_;  
};

UpgradeManager &upgrade_manager();

}  // namespace comm
}  // namespace livox

#endif  // LIVOX_UPGRADE_MANAGER_H_
