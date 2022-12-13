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

#include "logger_manager.h"
#include "file_manager.h"

#include "command_handler/general_command_handler.h"

#include "base/logging.h"
#include "comm/protocol.h"
#include "comm/generate_seq.h"

#include <iostream>
#include <iomanip>
#ifdef WIN32
#include<winsock2.h>
#else
#include <sys/socket.h>
#endif

namespace livox {
namespace lidar {

LoggerManager::LoggerManager()
    : log_enable_(false),
      log_save_path_(""),
      max_cache_size_(200 * 1024 * 1024),
      current_cache_size_(0),
      comm_port_(nullptr) {
}

LoggerManager& LoggerManager::GetInstance() {
  static LoggerManager logger_manager;
  return logger_manager;
}

bool LoggerManager::Init(std::shared_ptr<LivoxLidarLoggerCfg> lidar_logger_cfg_ptr) {
  if (lidar_logger_cfg_ptr == nullptr) {
    log_enable_.store(false);
    return true;
  }

  if (lidar_logger_cfg_ptr->lidar_log_enable == false) {
    log_enable_.store(false);
    return true;
  }

  comm_port_.reset(new CommPort());
  log_enable_.store(lidar_logger_cfg_ptr->lidar_log_enable);
  if (lidar_logger_cfg_ptr->lidar_log_cache_size != 0) {
    max_cache_size_ = (lidar_logger_cfg_ptr->lidar_log_cache_size) * 1024 * 1024;
  }
 
  if (!InitLoggerSavePath(lidar_logger_cfg_ptr->lidar_log_path)) {
    return false;
  }
  return true;
}

bool LoggerManager::InitLoggerSavePath(std::string log_save_path) {
  std::string log_root_dir = log_save_path + (log_save_path.back() == '/' ? "" : "/") + "lidar_log/";
  if (access(log_root_dir.c_str(), 0) != EXIT_SUCCESS) {
    if (!MakeDirecotory(log_root_dir)) {
      LOG_ERROR("Can't Create Dir {}", log_root_dir);
      return false;
    }
  } else {
    current_cache_size_ = GetDirTotalSize(log_root_dir);
    LOG_INFO("Livox Lidar Log Cache Size: {}, Max Cache Size: {}", (long long int)current_cache_size_, (long long int)max_cache_size_);
    if (current_cache_size_ >= max_cache_size_) {
      return false;
    }
  }
  log_save_path_ = log_root_dir;
  return true;
}

void LoggerManager::AddDevice(const uint32_t handle, const DetectionData* detection_data) {
  if (devices_info_.find(handle) == devices_info_.end()) {
    devices_info_[handle].sn = detection_data->sn;
    devices_info_[handle].dev_type = detection_data->dev_type;
    std::string lidar_ip = std::to_string(detection_data->lidar_ip[0]) + "." +
      std::to_string(detection_data->lidar_ip[1]) + "." +
      std::to_string(detection_data->lidar_ip[2]) + "." +
      std::to_string(detection_data->lidar_ip[3]);
    devices_info_[handle].lidar_ip = lidar_ip;
    devices_info_[handle].cmd_port = detection_data->cmd_port;
  }
}

void LoggerManager::RemoveDevice(const uint32_t handle) {
  if (devices_info_.find(handle) != devices_info_.end()) {
    devices_info_.erase(handle);
  }
}

livox_status LoggerManager::StartLogger(const uint32_t handle, const LivoxLidarLogType log_type,
    LivoxLidarLoggerStartCallback cb, void* client_data) {
  if (log_enable_.load() == false) {
    LOG_INFO("Disable logger.");
    return kLivoxLidarStatusSuccess;  
  }

  LOG_INFO("Start Logger handler: {}, log_type: {}", handle, log_type);

  EnableDeviceLoggerRequest enable_req = {};
  enable_req.log_type = static_cast<uint8_t>(log_type);
  enable_req.enable = true;

  return GeneralCommandHandler::GetInstance().SendLoggerCommand(handle, 
      kCommandIDLidarCollectionLog, (uint8_t*)&enable_req, sizeof(EnableDeviceLoggerRequest),
      MakeCommandCallback<LivoxLidarLoggerResponse>(cb, client_data));
}

void LoggerManager::Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size) {
  if (!log_enable_.load()) {
    return;
  }

  if (buf == nullptr || buf_size == 0) {
    return;
  }

  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  if (!(comm_port_->ParseCommStream((uint8_t*)buf, buf_size, &packet))) {
    LOG_INFO("Parse GeneralCommandHandler Command Stream failed.");
    return;
  }
  
  if (packet.cmd_id != kCommandIDLidarPushLog) {
    return;
  }

  auto data = static_cast<DeviceLoggerFilePushRequest*>((void *)packet.data);
  uint8_t flag = data->flag;
  if (flag & (1 << 1)) {
    OnLoggerCreate(handle, data);
    return;
  }

  if (flag & (1 << 2)) {
    OnLoggerStop(handle, data);
    return;
  }
  OnLoggerTransfer(handle, data);
}

void LoggerManager::OnLoggerCreate(const uint32_t handle, DeviceLoggerFilePushRequest* data) {
  if (handlers_.find(handle) == handlers_.end()) {
    if (devices_info_.find(handle) != devices_info_.end()) {
      auto broadcast_code = devices_info_[handle].sn;
      handlers_[handle] = std::make_shared<LoggerHandler>(log_save_path_, broadcast_code);
      handlers_[handle]->Init();
    }
  }

  auto & handler = handlers_[handle];

  handler->CreateFile(data);

  if (data->flag & 1) {
    DeviceLoggerFilePushReponse response = {};
    response.ret_code = 0x00;
    response.log_type = data->log_type;
    response.file_index = data->file_index;
    response.trans_index = data->trans_index;

    GeneralCommandHandler::GetInstance().SendLoggerCommand(handle,
      kCommandIDLidarPushLog, (uint8_t*)&response, sizeof(DeviceLoggerFilePushReponse),
      MakeCommandCallback<DeviceLoggerFilePushReponse>(nullptr, nullptr));
  }
}

void LoggerManager::OnLoggerStop(const uint32_t handle, DeviceLoggerFilePushRequest* data) {
  if (handlers_.find(handle) == handlers_.end()) {
    LOG_INFO("LogType: {} Stop! File doesn't create", (int)data->log_type);
    return;
  }

  auto & handler = handlers_[handle];
  handler->StopFile(data);

  if (data->flag & 1) {
    DeviceLoggerFilePushReponse response = {};
    response.ret_code = 0x00;
    response.log_type = data->log_type;
    response.file_index = data->file_index;
    response.trans_index = data->trans_index;

    GeneralCommandHandler::GetInstance().SendLoggerCommand(handle,
      kCommandIDLidarPushLog, (uint8_t*)&response, sizeof(DeviceLoggerFilePushReponse),
      MakeCommandCallback<DeviceLoggerFilePushReponse>(nullptr, nullptr));
  }
}

void LoggerManager::OnLoggerTransfer(const uint32_t handle, DeviceLoggerFilePushRequest* data) {
  if (handlers_.find(handle) == handlers_.end()) {
    LOG_ERROR("LogType : {} File doesn't create", (int)data->log_type);
    return;
  }
  auto & handler = handlers_[handle];

  if (!CheckAndUpdateCacheSize(data->data_length)) {
    handler->StopFile(data);
    return;
  }

  handler->WriteFile(data);

  if (data->flag & 1) {
    DeviceLoggerFilePushReponse response = {};
    response.ret_code = 0x00;
    response.log_type = data->log_type;
    response.file_index = data->file_index;
    response.trans_index = data->trans_index;

    GeneralCommandHandler::GetInstance().SendLoggerCommand(handle, //Send ACK
      kCommandIDLidarPushLog, (uint8_t*)&response, sizeof(DeviceLoggerFilePushReponse),
      MakeCommandCallback<DeviceLoggerFilePushReponse>(nullptr, nullptr));
  }
}

bool LoggerManager::CheckAndUpdateCacheSize(uint16_t data_length) {
  LOG_INFO("current cache size: {}, max cache size: {}", current_cache_size_, max_cache_size_);
  if (current_cache_size_ >= max_cache_size_) {
    return false;
  }
  current_cache_size_ += data_length;
  return true;
}

void LoggerManager::StopAllLogger() {
  if (!log_enable_.load()) {
    return;
  }
  for (auto it = devices_info_.begin(); it != devices_info_.end(); ++it) {
    const uint32_t handle = it->first;
    StopLogger(handle, kLivoxLidarRealTimeLog, nullptr, nullptr);
  }
}

livox_status LoggerManager::StopLogger(const uint32_t handle, const LivoxLidarLogType log_type,
    LivoxLidarLoggerStartCallback cb, void* client_data) {
  LOG_INFO("Stop Logger handler: {}, log_type: {}", handle, log_type);
  EnableDeviceLoggerRequest enable_req = {};
  enable_req.log_type = log_type;
  enable_req.enable = false;
  return GeneralCommandHandler::GetInstance().SendLoggerCommand(handle, 
      kCommandIDLidarCollectionLog, (uint8_t*)&enable_req, sizeof(EnableDeviceLoggerRequest),
      MakeCommandCallback<LivoxLidarLoggerResponse>(nullptr, nullptr));
}

void LoggerManager::LoggerStopCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    LOG_ERROR("Lidar:{} stop logger failed, the status:{}", handle, status);
    if (client_data) {
      LoggerManager* manager = static_cast<LoggerManager*>(client_data);
      manager->StopLogger(handle, kLivoxLidarRealTimeLog, LoggerManager::LoggerStopCallback, client_data);
    }
    return;
  }

  if (response == nullptr) {
    LOG_ERROR("Lidar:{} stop logger failed, the response is nullptr.", handle);
    LoggerManager* manager = static_cast<LoggerManager*>(client_data);
    manager->StopLogger(handle, kLivoxLidarRealTimeLog, LoggerManager::LoggerStopCallback, client_data);
    return;
  } 
  
  if (response->ret_code != 0) {
    LOG_ERROR("Lidar:{} stop logger failed, the ret_code:{}.", handle, response->ret_code);
    LoggerManager* manager = static_cast<LoggerManager*>(client_data);
    manager->StopLogger(handle, kLivoxLidarRealTimeLog, LoggerManager::LoggerStopCallback, client_data);
    return;
  }

  LOG_INFO("The lidar:{} stop logger succ.\n", handle);
  LoggerManager* manager = static_cast<LoggerManager*>(client_data);
  manager->RemoveDevice(handle);
}

void LoggerManager::Destory() {
  for (auto it = handlers_.begin(); it != handlers_.end(); ++it) {
    it->second->Destory();
  }
  
  if (!handlers_.empty()) {
    handlers_.clear();
  }

  StopAllLogger();
  log_enable_.store(false);
}

LoggerManager::~LoggerManager() {
  Destory();
}


} // namespace lidar
}  // namespace livox
