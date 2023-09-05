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
#include <condition_variable>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "logger_handler.h"

#include "base/io_thread.h"
#include "comm/define.h"
#include "base/network/network_util.h"

namespace livox {
namespace lidar {

#pragma pack(1)

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

#pragma pack()

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
  bool GetLogEnable();
  void AddDevice(const uint32_t handle, const DetectionData* detection_data);
  void RemoveDevice(const uint32_t handle);
  void Destory();

  livox_status StartLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data);
  livox_status StopLogger(const uint32_t handle, const LivoxLidarLogType log_type, LivoxLidarLoggerCallback cb, void* client_data);

  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);
  static void LoggerStopCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data);

private:
  bool InitLoggerSavePath(std::string log_root_path);
  void CreateDeviceDir(const uint32_t handle, const LidarDeviceInfo& device_info);

  void OnLoggerCreate(const uint32_t handle, DeviceLoggerFilePushRequest* data);
  void OnLoggerStopped(const uint32_t handle, DeviceLoggerFilePushRequest* data);
  void OnLoggerTransfer(const uint32_t handle, DeviceLoggerFilePushRequest* data);

  void CycleDelete();

  void StopAllLogger();
private:
  std::atomic<bool> log_enable_;
  std::atomic<bool> log_cycle_delete_enable_;
  std::string log_root_path_;
  uint64_t max_realtimelog_cache_size_;
  uint64_t max_exceptionlog_cache_size_;

  std::unique_ptr<CommPort> comm_port_;

  std::shared_ptr<std::thread> cycle_delete_thread_;
  bool cond_;
  std::mutex mutex_;
  std::condition_variable cv_;

  std::map<uint32_t, LidarDeviceInfo> devices_info_;
  std::map<uint32_t, std::shared_ptr<LoggerHandler>> handlers_;

  std::multimap<std::string, std::string> realtime_files_;
  std::multimap<std::string, std::string> exception_files_;

  std::atomic<bool> is_destroy_;
};

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_LOGGER_MANAGER_H_
