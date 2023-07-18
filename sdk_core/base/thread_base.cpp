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

#include "thread_base.h"
#include <thread>
#include <iostream>
#include <memory>

namespace livox {
namespace lidar {


ThreadBase::ThreadBase() : quit_(false) {}

bool ThreadBase::Start() {
  quit_ = false;
  thread_ = std::make_shared<std::thread>(&ThreadBase::ThreadFunc, this);
  return true;
}

ThreadBase::~ThreadBase() {
  if (thread_) {
    Join();
  }
}

void ThreadBase::Join() {
  quit_ = true;
  if (thread_ && thread_->joinable()) {
    thread_->join();
    thread_ = nullptr;
  } else {
    std::cout << "failed to join thread, joinable: " 
              << thread_->joinable() << std::endl;
    thread_ = nullptr;
  }
}

} // namespace lidar
}  // namespace livox
