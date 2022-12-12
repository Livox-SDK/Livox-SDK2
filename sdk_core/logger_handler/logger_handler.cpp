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

#include "logger_handler.h"

#include <iostream>
#include <sstream>
#include <iomanip>

#include "base/logging.h"
#include "command_handler/command_impl.h"
#include "device_manager.h"
#include "livox_lidar_def.h"
#include "file_manager.h"

namespace livox {
namespace lidar {

std::string GetCurFormatTime() {
  std::time_t t = std::time(nullptr);
  std::stringstream format_time;
  format_time << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S");
  return format_time.str();
}

void LoggerHandler::Init() {
  is_stop_write_.store(false);
  thread_ptr_ = std::make_shared<std::thread>(&LoggerHandler::SaveToFile, this);
}

void LoggerHandler::Destory() {
  if (thread_ptr_) {
    is_stop_write_.store(true);
    thread_ptr_->join();
    thread_ptr_ = nullptr;
  }

  for (auto &file: current_files_) {
   auto& fp = file.second.fp;
   if (fp) {
    std::fclose(fp);
    fp = nullptr;
   }
  } 
}

void LoggerHandler::SaveToFile() {
  while (!is_stop_write_.load()) {
    Write();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void LoggerHandler::CreateFile(DeviceLoggerFilePushRequest* req) {
  if (!req) {
    return;
  }
  std::string current_time_str = GetCurFormatTime();
  uint8_t log_type = req->log_type;
  std::ostringstream log_type_str;
  log_type_str << (int)log_type;
  std::string main_log_dir = log_save_path_ + (log_save_path_.back() == '/' ? "" : "/") + broadcast_code_;
  std::string sub_log_dir = main_log_dir + "/log_type_" + log_type_str.str();
  if (!IsDirectoryExits(main_log_dir)) {
    if (!MakeDirecotory(main_log_dir)) {
      LOG_ERROR("Can't Create Dir {}", main_log_dir);
      return;
    }
  }
  if (!IsDirectoryExits(sub_log_dir)) {
    if (!MakeDirecotory(sub_log_dir)) {
      LOG_ERROR("Can't Create Dir {}", sub_log_dir);
      return;
    }
  }

  std::string file_path =  sub_log_dir + "/" + std::to_string(req->file_index) + "_" + broadcast_code_ + "_" + current_time_str + ".dat";
  LOG_INFO("file path : {}", file_path);
  auto fp = std::fopen(file_path.c_str(), "ab");
  if (fp) {
    std::fwrite(req->data, 1, req->data_length, fp);
  }
  auto last_fp = current_files_[log_type].fp;
  if (last_fp) {
    std::fclose(last_fp);
  }

  current_files_[log_type].fp = fp;
  current_files_[log_type].file_index = req->file_index;
  LOG_INFO("Create File index: {}", (int)req->file_index);
  files_state_[log_type].total_file_num ++;
  files_state_[log_type].file_index = req->file_index;
}

void LoggerHandler::WriteFile(DeviceLoggerFilePushRequest* req) {
  uint8_t log_type = req->log_type;
  // update total trans size
  files_state_[log_type].transfer_size += req->data_length;
  //check file index is equal
  if (current_files_[log_type].file_index != req->file_index) {
    LOG_WARN("Log Type: {}, File Index error: last file index: {}, current file index: {}",
                (int)log_type,
                current_files_[log_type].file_index,
                req->file_index);
  }
  if (current_files_[log_type].trans_index + 1 != req->trans_index 
      && req->trans_index != 1) {
    LOG_WARN("Log Type: {}, Trans Index error: last trans index: {}, current trans index: {}",
            (int)log_type,
            current_files_[log_type].trans_index,
            req->trans_index);
  }

  current_files_[log_type].trans_index = req->trans_index;
  auto fp = current_files_[log_type].fp;

  if (fp) {
    LOG_INFO("Transform Data Length : {}", req->data_length);
    WriteBuffer write_buff = {};
    write_buff.buff_size = req->data_length;
    write_buff.buff.reserve(req->data_length);
    memcpy(write_buff.buff.data(), req->data, req->data_length);
    write_buff.fp = std::move(fp);

    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push(std::move(write_buff));
  }
}

void LoggerHandler::Write() {
  std::queue<WriteBuffer> queue;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue.swap(queue_);
  }

  while (!queue.empty()) {
    WriteBuffer& write_buff = queue.front();
    if (write_buff.fp) {
      std::fwrite(&write_buff.buff[0], 1, write_buff.buff_size, write_buff.fp);
      std::fflush(write_buff.fp);
    }
    queue.pop();
  }
}

void LoggerHandler::StopFile(DeviceLoggerFilePushRequest* req) {
  if (!req) {
    return;
  }
  auto fp = current_files_[req->log_type].fp;
  if (fp) {
    std::fclose(fp);
  }
  current_files_[req->log_type].fp = nullptr;
}

} // namespace lidar
}  // namespace livox
