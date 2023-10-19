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

#ifndef GENERAL_COMMAND_HANDLER_H_
#define GENERAL_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <condition_variable>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/define.h"

#include "livox_lidar_api.h"

#include "livox_lidar_def.h"
#include "device_manager.h"
#include "command_handler.h"

namespace livox {
namespace lidar {

typedef struct {
  std::string sn;
  std::string lidar_ip;
  uint8_t dev_type;
  std::atomic<bool> is_update_cfg;
  std::atomic<bool> is_get_loader_mode;
  std::atomic<bool> is_loader_mode;
  std::atomic<bool> is_callback;
} DeviceInfo;

class HapCommandHandle;

class GeneralCommandHandler : public noncopyable {
 private:
  GeneralCommandHandler();
  GeneralCommandHandler(const GeneralCommandHandler& other) = delete;
  GeneralCommandHandler& operator=(const GeneralCommandHandler& other) = delete;
 public:
  ~GeneralCommandHandler();
  void Destory();
  static GeneralCommandHandler& GetInstance();

  bool Init(const std::string& host_ip, const bool is_view, DeviceManager* device_manager);

  bool Init(std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr, DeviceManager* device_manager);
  
  // void SetDeviceManager(DeviceManager* device_manager);

  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);

  void Handler(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_port,
      uint8_t *buf, uint32_t buf_size);
  
  void CreateCommandHandler(const uint8_t dev_type);

  livox_status SendCommand(uint32_t handle, uint16_t command_id, uint8_t *data,
      uint16_t length, const std::shared_ptr<CommandCallback> &cb);
  
  livox_status SendLoggerCommand(uint32_t handle, uint16_t command_id, uint8_t *data,
                                 uint16_t length, const std::shared_ptr<CommandCallback> &cb);

  void CommandsHandle(TimePoint now);
  void AddCommand(const Command& command);
  void AddDetectedLidar(const std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);

  void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data) {
    livox_lidar_info_change_cb_ = cb;
    livox_lidar_info_change_client_data_ = client_data;
  }

  void SetLivoxLidarInfoCallback(LivoxLidarInfoCallback cb, void* client_data) {
    livox_lidar_info_cb_ = cb;
    livox_lidar_info_client_data_ = client_data;
  }

  void LivoxLidarAddCmdObserver(LivoxLidarCmdObserverCallBack cb, void* client_data) {
    cmd_observer_cb_ = cb;
    cmd_observer_client_data_ = client_data;
  }

  void LivoxLidarRemoveCmdObserver() {
    cmd_observer_cb_ = nullptr;
    cmd_observer_client_data_ = nullptr;
  }

  void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info);
  void UpdateLidarCfg(const uint8_t dev_type, const uint32_t handle, const uint16_t lidar_cmd_port);
  void LivoxLidarInfoChange(const uint32_t handle);
  void PushLivoxLidarInfo(const uint32_t handle, const std::string& info);
  bool GetQueryLidarInternalInfoKeys(const uint32_t handle, std::set<ParamKeyName>& key_sets);
  const LivoxLidarCfg& GetLidarCfg(const uint32_t handle);
  livox_status LivoxLidarRequestReset(uint32_t handle, LivoxLidarResetCallback cb, void* client_data);
  static void QueryFwTypeCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
 private:
  bool VerifyNetSegment(const DetectionData* detection_data);
  std::shared_ptr<CommandHandler> GetLidarCommandHandler(const uint8_t dev_type);
  std::shared_ptr<CommandHandler> GetLidarCommandHandler(const uint32_t handle);
  void HandleDetectionData(uint32_t handle, uint16_t lidar_port, const CommPacket& packet);
  void GetFirmwareType(const uint32_t handle, DeviceInfo& device_info);
  livox_status QueryFwType(const uint32_t handle);
  void UpdateFwType(const uint32_t handle, const uint8_t fw_type);
 private:
  DeviceManager* device_manager_;
  std::unique_ptr<CommPort> comm_port_;

  std::map<uint32_t, LivoxLidarCfg> custom_lidars_cfg_map_;

  std::mutex dev_type_mutex_;
  std::map<uint32_t, uint8_t> device_dev_type_;

  std::mutex devices_mutex_;
  std::map<uint32_t, DeviceInfo> devices_;

  std::mutex command_handle_mutex_;
  std::map<uint8_t, std::shared_ptr<CommandHandler>> lidars_command_handler_;

  std::mutex commands_mutex_;
  std::map<uint32_t, std::pair<Command, TimePoint> > commands_;

  LivoxLidarInfoChangeCallback livox_lidar_info_change_cb_;
  void* livox_lidar_info_change_client_data_;

  LivoxLidarInfoCallback livox_lidar_info_cb_;
  void* livox_lidar_info_client_data_;

  LivoxLidarCmdObserverCallBack cmd_observer_cb_{nullptr};
  void* cmd_observer_client_data_{nullptr};

  std::string detection_host_ip_;
  bool is_view_;
};

}  // namespace livox
} // namespace lidar

#endif  // GENERAL_COMMAND_HANDLER_H_
