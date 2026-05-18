/*
How to use?
- ./livox_lidar_quick
_start ../../../samples/livox_lidar_quick_start/mid360s_config.json
./livox_lidar_quick
_start ../../../samples/livox_lidar_quick_start/mid360_config.json

config내의 lidar IP를 각 모델 시리얼번호에 맞춰서 설정.
*/
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>

// ====== 설정 영역 ======
// 바꾸려는 센서의 "현재" IP
static const char* kTargetCurrentIp = "192.168.1.199";

// 바꾸려는 "새" IP / 마스크 / 게이트웨이
static const char* kNewLidarIp = "192.168.10.67";
static const char* kNewNetmask = "255.255.255.0";
static const char* kNewGateway = "192.168.10.1";
// =======================

// 이미 처리한 핸들 추적 (중복 호출 방지)
static std::set<uint32_t> g_processed_handles;
static std::mutex g_handles_mutex;

void RebootCallback(livox_status status, uint32_t handle,
                    LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) return;
  printf("[RebootCallback] handle:0x%08x ret_code:%u\n", handle,
         response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle,
                       LivoxLidarAsyncControlResponse* response,
                       void* client_data) {
  if (response == nullptr) return;
  printf(
      "[SetIpInfoCallback] handle:0x%08x status:%u ret_code:%u error_key:%u\n",
      handle, status, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    printf(">>> IP 변경 성공, 센서 재부팅 명령 전송...\n");
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  } else {
    printf(">>> IP 변경 실패!\n");
  }
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info,
                             void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }

  // handle을 IP 문자열로 변환
  struct in_addr addr;
  addr.s_addr = handle;
  const char* current_ip = inet_ntoa(addr);

  printf("[LidarInfoChange] handle:0x%08x IP:%s SN:%s\n", handle, current_ip,
         info->sn);

  // 중복 호출 방지
  {
    std::lock_guard<std::mutex> lock(g_handles_mutex);
    if (g_processed_handles.count(handle)) {
      printf("  -> 이미 처리한 핸들, skip\n");
      return;
    }
    g_processed_handles.insert(handle);
  }

  // 타겟 IP가 아니면 무시
  if (strcmp(current_ip, kTargetCurrentIp) != 0) {
    printf("  -> 타겟(%s) 아님, skip\n", kTargetCurrentIp);
    return;
  }

  printf("  -> 타겟 센서 발견! IP 변경 시작: %s -> %s\n", current_ip,
         kNewLidarIp);

  LivoxLidarIpInfo lidar_ip_info;
  memset(&lidar_ip_info, 0, sizeof(lidar_ip_info));
  strncpy(lidar_ip_info.ip_addr, kNewLidarIp,
          sizeof(lidar_ip_info.ip_addr) - 1);
  strncpy(lidar_ip_info.net_mask, kNewNetmask,
          sizeof(lidar_ip_info.net_mask) - 1);
  strncpy(lidar_ip_info.gw_addr, kNewGateway,
          sizeof(lidar_ip_info.gw_addr) - 1);

  SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type,
                               const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;
  std::cout << "[PushMsg] handle:0x" << std::hex << handle
            << " ip:" << inet_ntoa(tmp_addr) << std::endl;
  return;
}

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    printf("Usage: %s <config_path>\n", argv[0]);
    return -1;
  }
  const std::string path = argv[1];

  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  printf("=== IP 변경 도구 ===\n");
  printf("타겟 현재 IP: %s\n", kTargetCurrentIp);
  printf("타겟 새 IP:   %s\n", kNewLidarIp);
  printf("센서 검색 중... 60초 후 종료합니다.\n\n");

#ifdef WIN32
  Sleep(60000);
#else
  sleep(60);
#endif

  LivoxLidarSdkUninit();
  printf("종료.\n");
  return 0;
}