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

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <map>

#include <csignal>

std::condition_variable quit_condition;
std::mutex mtx;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  // printf("point cloud handle: %d, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}
     
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);
}

void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Start logger failed, the status :%d\n", status);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    printf("Start logger failed, the response is nullptr.\n");
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response->ret_code != 0) {
    printf("Start logger failed, the response ret_Code:%d.\n", response->ret_code);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  printf("The lidar[%u] start logger succ.\n", handle);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
}

void Stop(int signal) {
  quit_condition.notify_all();
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  SaveLivoxLidarSdkLoggerFile();

  //DisableLivoxSdkConsoleLogger();

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  // capture Ctrl + C signal.
  std::signal(SIGINT, Stop);

  std::unique_lock<std::mutex> lock(mtx);
  quit_condition.wait(lock);
  printf("Deivice Logger exit.\n");


  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}

