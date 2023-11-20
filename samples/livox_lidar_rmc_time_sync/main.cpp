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

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>

#include "synchro.h"

static void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}

static void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  Synchro& synchro = Synchro::GetInstance();
#ifdef WIN32
  synchro.SetPortName("COM3");
#else
  synchro.SetPortName("/dev/ttyUSB0");
#endif

/** Set Synchro's Baudrate: 9600. */
  synchro.SetBaudRate(BR9600);
/** Set Synchro's Parity: No parity (8N1). */
  synchro.SetParity(P_8N1);
/** SyncTimer Callback will trigger when Synchro's GPRMC/GNRMC signal is ready. */
  synchro.SetSyncTimerCallback([handle](const char* rmc, uint16_t rmc_length) {
    printf("GPRMC is:%s", rmc);
    SetLivoxLidarRmcSyncTime(handle, rmc, rmc_length, [](livox_status status, uint32_t handle, LivoxLidarRmcSyncTimeResponse* data, void* client_data){
      std::cout << "Lidar handle:" << handle << " response is: " << +data->ret << std::endl;
    }, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  });
  if (synchro.Start()) {
    printf("Synchro start success.");
  } else {
    printf("Synchro start failed.");
    return;
  }
}

static void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

#ifdef WIN32
  Sleep(300000);
#else
  sleep(300);
#endif
  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}
