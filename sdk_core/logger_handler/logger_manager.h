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

#ifndef LIVOX_LOGGER_MANAGER_H_
#define LIVOX_LOGGER_MANAGER_H_

#include <functional>
#include <memory>
#include <mutex>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "logger_handler.h"

#include "base/io_thread.h"
#include "comm/define.h"
#include "base/network/network_util.h"

namespace livox {
namespace lidar {

#pragma pack(1)

typedef struct {
  std::string sn;
  uint8_t dev_type;
  std::string lidar_ip;
  uint16_t cmd_port;
} LidarDeviceInfo;

typedef enum {
  kLidarLoggerCreate,
  kLidarLoggerStop,
  kLidarUnknown
} LogState;

struct LogInfo {
  LogInfo() {
    this->log_state = LogState::kLidarUnknown;
    this->total_log_size = 0;
  }
  LogState log_state;
  uint64_t total_log_size;
};

class LoggerManager {
 private:
  LoggerManager();
  LoggerManager(const LoggerManager& other) = delete;
  LoggerManager& operator=(const LoggerManager& other) = delete;
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;
  ~LoggerManager();
  static LoggerManager& GetInstance();

  bool Init(std::shared_ptr<LivoxLidarLoggerCfg> lidar_logger_cfg_ptr);
  void AddDevice(const uint32_t handle, const DetectionData* detection_data);
  void RemoveDevice(const uint32_t handle);
  void Destory();

  livox_status StartLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerStartCallback cb, void* client_data);

  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);
  static void LoggerStopCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data);
private:
  bool InitLoggerSavePath(std::string log_save_path);
  void CreateDeviceDir(const uint32_t handle, const LidarDeviceInfo& device_info);

  void OnLoggerCreate(const uint32_t handle, DeviceLoggerFilePushRequest* data);
  void OnLoggerStop(const uint32_t handle, DeviceLoggerFilePushRequest* data);
  void OnLoggerTransfer(const uint32_t handle, DeviceLoggerFilePushRequest* data);

  bool CheckAndUpdateCacheSize(uint16_t data_length);

  void StopAllLogger();
  livox_status StopLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerStartCallback cb, void* client_data);
private:
  std::atomic<bool> log_enable_;
  std::string log_save_path_;
  uint64_t max_cache_size_;
  uint64_t current_cache_size_;

  std::unique_ptr<CommPort> comm_port_;

  std::map<uint32_t, LidarDeviceInfo> devices_info_;
  std::map<uint32_t, std::shared_ptr<LoggerHandler>> handlers_;
};

} // namespace lidar
}  // namespace livox

#pragma pack()

#endif  // LIVOX_LOGGER_MANAGER_H_
