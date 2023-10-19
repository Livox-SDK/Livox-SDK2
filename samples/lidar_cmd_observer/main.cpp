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

#include "FastCRC/FastCRC.h"

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

  LivoxLidarAddCmdObserver([](const uint32_t handle, const LivoxLidarCmdPacket* data, void* client_data) {
    static FastCRC16 crc_16;
    static FastCRC32 crc_32;
    static const uint8_t offset_of_crc16 = offsetof(LivoxLidarCmdPacket, crc16_h);
    static const uint8_t offset_of_data  = offsetof(LivoxLidarCmdPacket, data);
    struct in_addr tmp_addr;
    tmp_addr.s_addr = handle;
    if (crc_16.ccitt(reinterpret_cast<const uint8_t*>(data), offset_of_crc16) != data->crc16_h) {
      printf("Error: liadr: %s, crc16 check failure.\n", inet_ntoa(tmp_addr));
      exit(-1);
    }
    if (crc_32.crc32(reinterpret_cast<const uint8_t*>(data) + offset_of_data, data->length - offset_of_data) != data->crc32_d) {
      printf("Error: liadr: %s, crc32 check failure.\n", inet_ntoa(tmp_addr));
      exit(-1);
    }
    printf("--------------------------------------------------\n");
    printf("liadr: %s, crc check success.\n", inet_ntoa(tmp_addr));
    printf("--------------------------------------------------\n");
  }, nullptr);

#ifdef WIN32
  Sleep(300000);
#else
  sleep(300);
#endif

  LivoxLidarRemoveCmdObserver();

  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}
