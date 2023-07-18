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

#ifdef WIN32
  #include <direct.h>
#else
  #include <dirent.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <unistd.h>
#endif

#include <algorithm>
#include <string>
#include <map>

#include "base/logging.h"

namespace livox {
namespace lidar {

constexpr uint16_t kLengthOfTimeInFilename = 19;
constexpr bool kFOk = 0;

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

bool GetFileNames(const std::string& dir_name, std::multimap<std::string, std::string> &filenames) {
  uint64_t total_size = 0;
  WIN32_FIND_DATAA data;
  HANDLE sh = NULL;

  sh = FindFirstFileA((dir_name +"\\*").c_str(), &data);
  if (sh == INVALID_HANDLE_VALUE ) {
    LOG_ERROR("get directory stat error");
    return false;
  }

  do {
    // skip current and parent
    if (std::string(data.cFileName).compare(".") == 0 || std::string(data.cFileName).compare("..") == 0) {
      continue;
    }
    // if found object is ...
    if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == FILE_ATTRIBUTE_DIRECTORY) {
      // directory, then search it recursievly
      GetFileNames(dir_name +"\\"+ data.cFileName, filenames);
    } else {
      // otherwise get file name and add it to std::map<std::string, std::string> &filenames
      if(!StoreFileName(data.cFileName, filenames)) {
        LOG_ERROR("StoreFileName {} failed", data.cFileName);
      }
    }
  } while (FindNextFileA(sh, &data));

  FindClose(sh);
  return true;
}

bool ChangeHiddenFiles(const std::string& dir_name) {
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
        ChangeHiddenFiles(dir_name +"\\"+ data.cFileName);
      } else {
        // otherwise get file name and change it to normal file name
        char first_element = '.';
        if (data.cFileName[0] != first_element) {
          continue;
        }
        std::string filename(data.cFileName);
        if (filename.empty()) {
          continue;
        }
        if(access((dir_name + "\\" + filename).c_str(), kFOk) != 0) {
          LOG_WARN("The file to be renamed : {} does not exist ", filename);
          continue;
        }
        std::string file_name_cut = filename.substr(1);
        if(access((dir_name + "\\" + file_name_cut).c_str(), kFOk) == 0) {
          if (remove((dir_name + "\\" + file_name_cut).c_str()) != 0) {
            LOG_WARN("Failed to remove the existing file: {}. errno: {}", file_name_cut, errno);
          }
        }
        if (rename((dir_name + "\\" + filename).c_str(), (dir_name + "\\" + file_name_cut).c_str()) != 0) {
          LOG_WARN("Rename hidden file {} failed. errno: {}", filename, errno);
        }
      }
    }
  } while (FindNextFileA(sh, &data));

  FindClose(sh);
  return true;
}

#else
uint64_t GetDirTotalSize(const std::string& dir_name) {
  struct stat dir_stat;
  if (stat(dir_name.c_str(), &dir_stat) != EXIT_SUCCESS) {
      LOG_ERROR("get directory stat error");
      return 0;
  }  
  if (S_ISREG(dir_stat.st_mode)) {
    return dir_stat.st_size;
  }  
  if (!S_ISDIR(dir_stat.st_mode)) {
    LOG_WARN("unknown directory type: {}", dir_name);
    return 0;
  }

  uint64_t total_size = 0;
  DIR *dirp = opendir(dir_name.c_str());
  if (!dirp) {
    LOG_ERROR("opendir: {} failed", dir_name);
    return 0;
  }
  struct dirent *dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    // ignore . and ..
    if (strcmp(".", dp -> d_name) == EXIT_SUCCESS || strcmp("..", dp -> d_name) == EXIT_SUCCESS) {
        continue;
    }    
    std::string sub_dir_name = dir_name;
    sub_dir_name.append("/").append(dp -> d_name);
    total_size += GetDirTotalSize(sub_dir_name.c_str());
  }
  closedir(dirp);
  
  return total_size;
}

bool GetFileNames(const std::string& dir_name, std::multimap<std::string, std::string> &filenames) {
  DIR *dirp = opendir(dir_name.c_str());
  if (!dirp) {
    LOG_ERROR("opendir: {} failed", dir_name);
    return false;
  }

  struct dirent *dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    if (strcmp(dp->d_name, ".")   == 0 || 
        strcmp(dp->d_name, "..")  == 0 ||
        dp->d_type == DT_LNK) {
      continue;
    }
    
    if (dp->d_type == DT_REG) {
      if (dp->d_name[0] == '.') {
        continue;
      }
      if(!StoreFileName(dp->d_name, filenames)) {
        LOG_ERROR("StoreFileName {} failed", dp->d_name);
      }
    } else if (dp->d_type == DT_DIR) {
      std::string sub_dir_name = dir_name;
      sub_dir_name.append("/").append(dp -> d_name);
      GetFileNames(sub_dir_name, filenames);
    }
  }
  closedir(dirp);

  return true;
}

bool ChangeHiddenFiles(const std::string& dir_name) {
  if (dir_name.empty()) {
    return false;
  }
  DIR *dirp = opendir(dir_name.c_str());
  if (!dirp) {
    LOG_ERROR("opendir: {} failed", dir_name);
    return false;
  }

  struct dirent *dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    if (strcmp(dp->d_name, ".") == 0 || strcmp(dp->d_name, "..") == 0) {
      continue;
    } else if (dp->d_type == DT_REG) {
      char first_element = '.';
      if (dp->d_name[0] != first_element) {
        continue;
      }
      std::string filename(dp->d_name);
      if (filename.empty()) {
        continue;
      }
      if(access((dir_name + "/" + filename).c_str(), kFOk) != 0) {
        LOG_WARN("The file to be renamed : {} does not exist ", filename);
        continue;
      }
      std::string file_name_cut = filename.substr(1);
      if(access((dir_name + "/" + file_name_cut).c_str(), kFOk) == 0) {
        if (remove((dir_name + "/" + file_name_cut).c_str()) != 0) {
          LOG_WARN("Failed to remove the existing file: {}. errno: {}", file_name_cut, errno);
        }
      }
      if (rename((dir_name + "/" + filename).c_str(), (dir_name + "/" + file_name_cut).c_str()) != 0) {
        LOG_WARN("Rename hidden file {} failed. errno: {}", filename, errno);
      }
    } else if (dp->d_type == DT_LNK) {
      continue;
    } else if (dp->d_type == DT_DIR) {
      std::string sub_dir_name = dir_name;
      sub_dir_name.append("/");
      sub_dir_name.append(dp -> d_name);
      ChangeHiddenFiles(sub_dir_name);
    }

  }
  closedir(dirp);

  return true;
}

bool DeleteHidFiles(const std::string& dir_name) {
  DIR *dirp = opendir(dir_name.c_str());
  if (!dirp) {
    LOG_ERROR("opendir: {} failed", dir_name);
    return false;
  }

  struct dirent *dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    if (strcmp(dp->d_name, ".") == 0 || strcmp(dp->d_name, "..") == 0) {
      continue;
    } else if (dp->d_type == DT_REG) {
      char first_element = '.';
      if (dp->d_name[0] != first_element) {
        continue;
      }
      std::string filename(dp->d_name);
      remove((dir_name + "/" + filename).c_str());
    } else if (dp->d_type == DT_LNK) {
      continue;
    } else if (dp->d_type == DT_DIR) {
      std::string sub_dir_name = dir_name;
      sub_dir_name.append("/");
      sub_dir_name.append(dp -> d_name);
      DeleteHidFiles(sub_dir_name);
    }

  }
  closedir(dirp);
  return true;
}
#endif

bool ChangeCurrentFileName(const std::string& dir_name, std::string file_name) {
  if (file_name.empty()) {
    return false;
  }
  if (file_name.at(0) != '.') {
    return false;
  }
  if(access((dir_name + "/" + file_name).c_str(), kFOk) != 0) {
    LOG_WARN("The file to be renamed : {} does not exist ", file_name);
    return false;
  }
  std::string file_name_cut = file_name.substr(1);
  if(access((dir_name + "/" + file_name_cut).c_str(), kFOk) == 0) {
    if (remove((dir_name + "/" + file_name_cut).c_str()) != 0) {
      LOG_WARN("Failed to remove the existing file: {}. errno: {}", file_name_cut, errno);
    }
  }
  if (rename((dir_name + "/" + file_name).c_str(), (dir_name + "/" + file_name_cut).c_str()) != 0) {
    LOG_WARN("Rename hidden file {} failed. errno: {}", file_name, errno);
    return false;
  }
  return true;
}

bool StoreFileName(const char* filename, std::multimap<std::string, std::string> &filenames) {
  std::string str_filename(filename); 
  if (str_filename.empty()) {
    return false;
  }
  std::string record_time = str_filename.substr(0, kLengthOfTimeInFilename);
  filenames.insert(std::make_pair(record_time, str_filename));
  return true;
}

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
