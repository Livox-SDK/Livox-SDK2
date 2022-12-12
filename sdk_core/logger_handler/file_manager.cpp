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

#include "file_manager.h"
#include "base/logging.h"
#ifdef WIN32
  #include <direct.h>
#else
  #include <dirent.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <unistd.h>
#endif

namespace livox {
namespace lidar {

#ifdef WIN32
uint64_t GetDirTotalSize(const std::string& dir_name) {
  uint64_t total_size = 0;
  WIN32_FIND_DATAA data;
  HANDLE sh = NULL;

  sh = FindFirstFileA((dir_name +"\\*").c_str(), &data);
  if (sh == INVALID_HANDLE_VALUE ) {
    LOG_ERROR("get directory stat error");
    return 0;
  }

  do {
    // skip current and parent
    if (std::string(data.cFileName).compare(".") != 0 && std::string(data.cFileName).compare("..") != 0)
    {
      // if found object is ...
      if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == FILE_ATTRIBUTE_DIRECTORY) {
        // directory, then search it recursievly
        total_size += GetDirTotalSize(dir_name +"\\"+ data.cFileName);
      } else {
        // otherwise get object size and add it to directory size
        total_size += (__int64) (data.nFileSizeHigh * (MAXDWORD ) + data.nFileSizeLow);
      }
    }
  } while (FindNextFileA(sh, &data));

  FindClose(sh);
  return total_size;
}

#else
uint64_t GetDirTotalSize(const std::string& dir_name) {
  struct stat dir_stat;
  uint64_t total_size = 0;
  if (stat(dir_name.c_str(), &dir_stat) != EXIT_SUCCESS) {
      LOG_ERROR("get directory stat error");
      return 0;
  }
  
  if (S_ISREG(dir_stat.st_mode)) {
      total_size += dir_stat.st_size;
  } else if (S_ISDIR(dir_stat.st_mode)) {
      DIR *dirp = opendir(dir_name.c_str());
      if (!dirp) {
          LOG_ERROR("opendir: {} failed", dir_name);
          return 0;
      }
      struct dirent *dp = nullptr;
      while ((dp = readdir(dirp)) != nullptr) {
          // ignore . and ..
          bool is_current_directory = (strcmp(".", dp -> d_name) == EXIT_SUCCESS);
          if (is_current_directory) {
              continue;
          }
          
          bool is_up_directory = (strcmp("..", dp -> d_name) == 0);
          if (is_up_directory) {
              continue;
          }
          
          std::string sub_dir_name = dir_name;
          sub_dir_name.append("/");
          sub_dir_name.append(dp -> d_name);
          
          total_size += GetDirTotalSize(sub_dir_name.c_str());
      }
      closedir(dirp);
  } else {
      LOG_ERROR("Unknow File Type!");
      return 0;
  }
  return total_size;
}
#endif

bool MakeDirecotory(std::string dir) {
  int flag = -1;
#ifdef WIN32
  flag = mkdir(dir.c_str());
#else
  flag = mkdir(dir.c_str(), 0777);
#endif // WIN32
  return (flag == 0);
}

bool IsDirectoryExits(std::string dir) {
  return access(dir.c_str(), 0) == EXIT_SUCCESS;
}

} // namespace lidar
}  // namespace livox
