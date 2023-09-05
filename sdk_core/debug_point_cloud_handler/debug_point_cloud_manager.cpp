#include "debug_point_cloud_manager.h"

#include "spdlog/fmt/fmt.h"

#include <iostream>
#include <cstdio>

namespace livox {
namespace lidar {

DebugPointCloudManager::DebugPointCloudManager() {}

DebugPointCloudManager::~DebugPointCloudManager() {
  enable_.store(false);
}

DebugPointCloudManager& DebugPointCloudManager::GetInstance() {
  static DebugPointCloudManager singleton;
  return singleton;
}

void DebugPointCloudManager::AddDevice(const uint32_t handle, const DetectionData* detection_data) {
  if (devices_info_.find(handle) == devices_info_.end()) {
    std::string ip = fmt::format("{}.{}.{}.{}", detection_data->lidar_ip[0],
                                                detection_data->lidar_ip[1],
                                                detection_data->lidar_ip[2],
                                                detection_data->lidar_ip[3]);
    devices_info_.emplace(handle, LidarDeviceInfo{detection_data->sn, detection_data->dev_type, ip, detection_data->cmd_port});
  }
}

void DebugPointCloudManager::Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size) {
  if (!enable_.load()) return;
  if (handlers_.find(handle) == handlers_.end()) {
    auto it = devices_info_.find(handle);
    if (it != devices_info_.end()) {
      handlers_.emplace(handle, std::make_shared<DebugPointCloudHandler>(handle, it->second.sn, it->second.dev_type, path_));
      handlers_[handle]->Enable(enable_.load());
    }
  }
  handlers_[handle]->StoreData(buf, buf_size);
}

bool DebugPointCloudManager::Enable(bool enable) {
  enable_.store(enable);
  for (auto& kv : handlers_) {
    kv.second->Enable(enable);
    if (!enable) {
      kv.second = nullptr;
    }
  }
  return true;
}

bool DebugPointCloudManager::SetStorePath(std::string path) {
  if (path.back() == '/') path.pop_back();
  path_ = path;
  return true;
}


} // namespace lidar
}  // namespace livox