# 1 Introduction

Livox SDK2 is a software development kit designed for all Livox lidars. It is developed based on C/C++ following Livox SDK2 Communication Protocol, and provides easy-to-use C style APIs. With the Livox SDK2, users can quickly connect to the Livox Lidars and receive point cloud data.

Livox SDK2 consists of [Livox SDK2 core code](sdk_core/src/), [Livox SDK2 APIs](sdk_core/include/livox_lidar_sdk.h) and one [sample](sample/livox_lidar_quick_start/).

## Prerequisites
* x86
    * Ubuntu 20.04 / Ubuntu 18.04 / Ubuntu 16.04

* ARM (Nvidia TX2)
    * Ubuntu 18.04

* Windows 10 + Visual Studio 2019
* Compiler supporting C++11


## Livox SDK2 API

Livox SDK2 API provides a set of C-style APIs, which can be conveniently integrated in C/C++ programs. Please refer to the **[Livox SDK2 APIs](sdk_core/include/livox_lidar_sdk.h)**.

## Livox SDK2 Communication Protocol

Livox SDK2 communication protocol opens to all users. It is the communication protocol between user programs and livox products. The protocol consists of control commands and data format, please refer to the documents below:

**HAP(TX/T1)**:

* [HAP SDK protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP>) (中文)
* [HAP SDK protocol](<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English)>) (English)


# 2 How to installation

## 2.1 Ubuntu 20.04 / 18.04 / 16.04 LTS
Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+

Install the **CMake** using apt:

```shell
$ sudo apt install cmake
```

Download and compile the Livox-SDK2:

```shell
$ git clone https://github.com/Livox-SDK/Livox-SDK2.git
$ cd ./Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

## 2.2 Windows 10

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

## 2.3 ARM-Linux Compile

The procedure of compile Livox-SDK2 in ARM-Linux are shown below.

Dependencies:

* [CMake 3.0.0+](https://cmake.org/)
* gcc 4.8.1+

Install the **CMake** using apt:

```shell
$ sudo apt install cmake
```

Download and compile the Livox-SDK2:

```shell
$ git clone https://github.com/Livox-SDK/Livox-SDK2.git
$ cd ./Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

# 3 How to Run the Sample

## 3.1 Ubuntu 20.04 / 18.04 / 16.04LTS
Connect to the Lidar(s), and run the program '**livox_lidar_quick_start**' :

```shell
$ cd sample/livox_lidar_quick_start && ./livox_lidar_quick_start ../../../sample/livox_lidar_quick_start/config.json
```

For how to configure the config.json file, pealse refer to [config.json](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>)


## 3.2 Windows 10
After compiling the Livox SDK2 as shown in section 2.1.2, you can find '**livox_lidar_quick_start.exe**' in the directory of '**Livox-SDK2\\build\\sample\\livox_lidar_quick_start\\Debug\\**'.

Copy the config file '**Livox-SDK2\\sample\\livox_lidar_quick_start\\config.json**' into the directory containing the program '**livox_lidar_quick_start.exe**', and run:


```cmd
> livox_lidar_quick_start.exe config.json
```

Then you can see the information as below:

![](doc/images/sdk_init.png)

For how to configure the config.json file, pealse refer to [config.json](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>)

**Note** : In the example, the config file '**config.json**' is copied to the same directory containing the program '**livox_lidar_quick_start.exe**'.

## 3.3 ARM-Linux

Connect to the Lidar(s), and run the '**livox_lidar_quick_start**':

```shell
$ cd sample/livox_lidar_quick_start && ./livox_lidar_quick_start ../../../sample/livox_lidar_quick_start/config.json
```

About the configuration of the '**config.json**' file, please refer to [HAP Config File Description](<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>).


# 4 Support

You can get support from Livox via:

* Send an email to cs@livoxtech.com, appended with detailed description for your problem and your setup;
* Raise a github issue

