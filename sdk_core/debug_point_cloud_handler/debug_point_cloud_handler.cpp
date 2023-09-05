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

#include "debug_point_cloud_handler.h"

#include "FastCRC/FastCRC.h"
#include "spdlog/fmt/fmt.h"

#include <algorithm>
#include <iostream>
#include <chrono>

namespace livox {
namespace lidar {

DebugPointCloudHandler::DebugPointCloudHandler(std::uint32_t handle, std::string sn, std::uint8_t dev_type, std::string path)
                        : handle_(handle), sn_(sn), dev_type_(dev_type), file_size_(0) {
  if (path.back() == '/') path.pop_back();
  file_path_ = path;
}

DebugPointCloudHandler::~DebugPointCloudHandler() {
  if (thread_ptr_) {
    enable_.store(false);
    thread_ptr_->join();
    thread_ptr_ = nullptr;
  }
}

const std::string GetCurrentSystemTime() {
	auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::tm now_tm;
  #ifdef WIN32
  localtime_s(&now_tm, &now);
  #else
  localtime_r(&now, &now_tm);
  #endif
  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", &now_tm);
	return buffer;
}

bool DebugPointCloudHandler::StoreData(uint8_t* buf, uint32_t buf_size) {
  if (buf) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    std::copy(buf, buf + buf_size, std::back_inserter(data_));
    lock.unlock();
    cv_.notify_one();
    return true;
  }
  return false;
}

void DebugPointCloudHandler::WriteData() {
  while (enable_.load()) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    cv_.wait_for(lock, std::chrono::seconds(1), [this]{return !data_.empty() || !enable_.load();});
    if (!enable_.load()) return;
    if (file_size_ >=  max_file_size_) {
      LOG_WARN("{} file size over 4 GB", file_name_);
      return;
    }
    if (!file_handle_) {
      file_handle_ = std::make_shared<std::ofstream>();
      file_name_ = fmt::format("lidar_{}_{}.LivoxDebugPointCloudData", handle_, GetCurrentSystemTime());
      file_handle_->open(fmt::format("{}/{}",file_path_, file_name_), std::ios::binary);
      if (!file_handle_->is_open()) {
        LOG_ERROR("filed to open {} path", file_path_);
        return;
      }
      // write file header
      FastCRC16 crc_16;
      LivoxLidarDebugPointCloudFileHeader file_header;
      file_header.file_ver  = {0x01};
      file_header.dev_type  = {dev_type_};
      file_header.data_type = {0x01};
      memcpy(file_header.sn, sn_.c_str(), sizeof(file_header.sn));
      memset(file_header.rsvd, 0, sizeof(file_header.rsvd));
      file_header.crc16 = crc_16.ccitt(reinterpret_cast<const uint8_t*>(&file_header),
                                      offsetof(LivoxLidarDebugPointCloudFileHeader, crc16));

      file_handle_->write(reinterpret_cast<char*>(&file_header), sizeof(file_header));
      file_size_ += sizeof(file_header);
    }
    file_handle_->write(reinterpret_cast<char*>(data_.data()), data_.size());
    file_size_ += data_.size();
    data_.resize(0);
  }
}

bool DebugPointCloudHandler::Enable(bool enable) {
  enable_.store(enable);
  if (enable) {
    thread_ptr_ = std::make_shared<std::thread>(&DebugPointCloudHandler::WriteData, this);
  }
  return true;
}

} // namespace lidar
}  // namespace livox
