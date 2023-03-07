# Livox SDK2

## Introduction

Livox SDK2 is a software development kit designed for all Livox lidars such as HAP and Mid-360. It is developed based on C/C++ following Livox SDK2 Communication Protocol, and provides easy-to-use C style APIs. With the Livox SDK2, users can quickly connect to the Livox Lidars and receive point cloud data.

Livox SDK2 consists of [Livox SDK2 core code](sdk_core/), [Livox SDK2 APIs](include/livox_lidar_api.h) and three [samples](samples/).

### Prerequisites

* OS:
  * Linux: Ubuntu 18.04 / 20.04, ...
  * Windows 10 / 11

* Tools:
  * compilers that support C++11
  * cmake 3.0+

* Arch:
  * x86
  * ARM

### Livox SDK2 API

Livox SDK2 API provides a set of C-style APIs, which can be conveniently integrated in C/C++ programs. Please refer to the **[Livox SDK2 APIs](include/livox_lidar_api.h)**.

### Livox SDK2 Communication Protocol

Livox SDK2 communication protocol opens to all users. It is the communication protocol between user programs and livox products. The protocol consists of control commands and data format, please refer to the documents below:

**HAP(TX/T1)**:

* [HAP Communication protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP>) (Chinese)
* [HAP Communication protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English)>) (English)

**Mid-360**:

* [Mid-360 Communication protocol](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/tutorials/new_product/mid360/mid360.html) (Chinese)
* [Mid-360 Communication protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html) (English)

## Installation

### Ubuntu 20.04 / 18.04

Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+

Install the **CMake** using apt:

```shell
$ sudo apt install cmake
```

Compile and install the Livox-SDK2:

```shell
$ git clone https://github.com/Livox-SDK/Livox-SDK2.git
$ cd ./Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

**Note :**
The generated shared library and static library are installed to the directory of "/usr/local/lib". The header files are installed to the directory of "/usr/local/include".

Uninstall sdk:

```shell
$ sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
$ sudo rm -rf /usr/local/include/livox_lidar_*
```

### Windows 10

Dependencies:

* Visual Studio 2019
* [CMake 3.0.0+](https://cmake.org/)

Preparation:

```cmd
> git clone https://github.com/Livox-SDK/Livox-SDK2.git
> cd Livox-SDK2
> md build && cd build
```

Generate a 64-bit project:

```cmd
> cmake .. -G "Visual Studio 16 2019" -A x64
```

Generate a 32-bit project:

```cmd
> cmake .. -G "Visual Studio 16 2019" -A Win32
```

Compiling:

You can now compile the Livox-SDK2 in Visual Studio 2019.


## How to Run the Sample

Livox SDK2 includes three samples, which are "livox_lidar_quick_start", "logger" and "multi_lidars_upgrade".

### Livox lidar quick start

#### Ubuntu 20.04 / 18.04

Connect to the Lidar(s), and run the program '**livox_lidar_quick_start**' :

```shell
$ cd samples/livox_lidar_quick_start && ./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/config.json
```

#### Windows 10
After compiling the Livox SDK2 as shown in Installation above, you can find '**livox_lidar_quick_start.exe**' in the directory of '**Livox-SDK2\\out\\build\\x64-Release\\samples\\livox_lidar_quick_start\\**'.

Copy the config file '**Livox-SDK2\\sample\\livox_lidar_quick_start\\config.json**' into the directory containing the program '**livox_lidar_quick_start.exe**', and run:


```cmd
> livox_lidar_quick_start.exe config.json
```

Then you can see the information as below:

```shell
> [info] Data Handle Init Succ.  [data_handler.cpp] [Init] [42]
> [info] Create detection channel detection socket:0  [device_manager.cpp] [CreateDetectionChannel] [232]
```

In the example, the config file '**config.json**' is copied to the same directory containing the program '**livox_lidar_quick_start.exe**'.

**Note** : 
1. When using HAP lidar / Mid-360 lidar, you can also 
replace "config.json" with "hap_config.json" / "mid360_config.json" in the command above.
2. For how to configure config.json / hap_config.json / mid360_config.json file, pealse refer to [config file description](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>).

### Logger

#### Parameters Configuration

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| lidar_log_enable | Enable or disable lidar logger. <br>Logger is enabled by default. | true    |
| lidar_log_cache_size_MB  | Set lidar log cache size. The unit is MB .<br> | 500       |
| lidar_log_path  | Set the save path of lidar log file.<br>The log file is saved in the current path by default.  | "./"       |

These Parameters are located in config.json / hap_config.json / mid360_config.json files

#### Run the 'logger'

##### Ubuntu 20.04 / 18.04

Connect to the Lidar(s), and run the program '**logger**' :

```shell
$ cd samples/logger && ./logger ../../../samples/logger/config.json
```

##### Windows 10

After compiling the Livox SDK2 as shown in Installation above, you can find '**logger.exe**' in the directory of '**Livox-SDK2\\out\\build\\x64-Release\\samples\\logger\\**'.

Copy the config file '**Livox-SDK2\\sample\\logger\\config.json**' into the directory containing the program '**logger.exe**', and run:


```cmd
> logger.exe config.json
```

In the example, the config file '**config.json**' is copied to the same directory containing the program '**logger.exe**'.

**Note** : 
1. When using HAP lidar / Mid-360 lidar, you can also 
replace "config.json" with "hap_config.json" / "mid360_config.json" in the command above.
2. For how to configure config.json / hap_config.json / mid360_config.json file, pealse refer to [config file description](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>).

### Multi-lidars upgrade

#### Ubuntu 20.04 / 18.04

Connect to the Lidar(s), and run the program '**multi_lidars_upgrade**' :

```shell
$ cd samples/multi_lidars_upgrade && ./multi_lidars_upgrade ../../../samples/multi_lidars_upgrade/config.json [firmware file path]
```
After executing the above command, Lidar stops and the firmware upgrade starts. 
The Lidar(s) upgrade takes a while and the upgrade progress is printed on termial. Also "upgrade successfully" will be printed on terminal when finishing upgrading.

#### Windows 10
After compiling the Livox SDK2 as shown in Installation above, you can find '**multi_lidars_upgrade.exe**' in the directory of '**Livox-SDK2\\out\\build\\x64-Release\\samples\\multi_lidars_upgrade\\**'.

Copy the config file '**Livox-SDK2\\sample\\multi_lidars_upgrade\\config.json**' and firmware file into the directory containing the program '**multi_lidars_upgrade.exe**', and run:

```cmd
> multi_lidars_upgrade.exe config.json [firmware file name]
```

In the example, the config file '**config.json**' is copied to the same directory containing the program '**multi_lidars_upgrade.exe**'.

**Note** : 
1. When using HAP lidar / Mid-360 lidar, you can also 
replace "config.json" with "hap_config.json" / "mid360_config.json" in the command above.
2. For how to configure config.json / hap_config.json / mid360_config.json file, pealse refer to [config file description](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>).


## Support

You can get support from Livox via:

* Send an email to cs@livoxtech.com, appended with detailed description for your problem and your setup;
* Raise a github issue
