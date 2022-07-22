[TOC]

# 1 Introduction

Livox SDK2 is the software development kit designed for all Livox products. It is developed based on C/C++ following Livox SDK2 Communication Protocol, and provides easy-to-use C style API. With Livox SDK2, users can quickly connect to Livox products and receive point cloud data. 

Livox SDK2 consists of Livox SDK2 communication protocol, Livox SDK2 core, Livox SDK2 API(livox_lidar_sdk.h), Linux sample(livox_lidar_quick_start)

## Prerequisites
* x86
  * Ubuntu 20.04 /Ubuntu 18.04/Ubuntu 16.04

* ARM (Nvidia TX2) 
  * Ubuntu 18.04

* Windows 7/10, Visual Studio 2019
* C++11 compiler 



# 2 Livox SDK API

Livox SDK2 API provides a set of C style functions which can be conveniently integrated in C/C++ programs. Please refer the below documents for further information : 

<!--HAP(TX): [Livox SDK2 API Reference](https://livox-sdk.github.io/Livox-SDK-Doc/)-->

<!--HAP(T1): [Livox SDK2 API Reference](https://livox-sdk.github.io/Livox-SDK-Doc/)-->



## 2.1 Installation
The installation procedures in Ubuntu 20.04 /Ubuntu 18.04/Ubuntu 16.04 LTS and Windows 7/10 are shown here as examples. 
### 2.1.1 Ubuntu 20.04/18.04/16.04 LTS
#### Dependencies
Livox SDK2 requires [CMake 3.0.0+](https://cmake.org/) as dependencies. You can install these packages using apt:  
```
sudo apt install cmake
```
#### Compile Livox SDK2

In the Livox SDK2 directory, run the following commands to compile the project:   **《赵确认》**
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cmake .. && make -j
sudo make install
```

### 2.1.2 Windows 7/10

#### Dependencies
Livox SDK2 supports Visual Studio 2019 and requires install [CMake 3.0.0+](https://cmake.org/) as dependencies.  

In the Livox SDK2 directory, run the following commands to create the Visual Studio solution file. 

For Viusal Studio 2019:

```
cd Livox-SDK2/build
```
Generate the 32-bit project:

```
cmake .. -G "Visual Studio 16 2019" -A Win32
```
Generate the 64-bit project:
```
cmake .. -G "Visual Studio 16 2019" -A x64
```

#### Compile Livox SDK2
You can now compile the Livox SDK2 in Visual Studio.

### 2.1.3 ARM-Linux Cross Compile

The procedure of cross compile Livox-SDK2 in ARM-Linux are shown below.

#### Dependencies

Host machine requires install cmake. You can install these packages using apt:

```
sudo apt install cmake
```

#### Cross Compile Toolchain

If your ARM board vendor provides a cross compile toolchain, you can skip the following step of installing the toolchain and use the vendor-supplied cross compile toolchain instead.

The following commands will install C and C++ cross compiler toolchains for 32bit and 64bit ARM board. You need to install the correct toolchain for your ARM board. For 64bit SoC ARM board, only install 64bit toolchain, and for 32bit SoC ARM board, only install 32bit toolchain.

Install **ARM 32 bits cross compile toolchain**：（**测试确认**，还没有这个环境）

```
 sudo apt-get install gcc-arm-linux-gnueabi g++-arm-linux-gnueabi
```

Install **ARM 64 bits cross compile toolchain**   **(不能确定，赵确认)**

```
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

#### Cross Compile Livox-SDK2   （**测试确认**，赵确认，测试编译的是ROS1）

For  **ARM 32 bits toolchain**，In the Livox SDK2 directory，run the following commands to cross compile the project:**(没有环境)**

```
cd Livox-SDK
cd build && \
cmake .. -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_C_COMPILER=arm-linux-gnueabi-gcc -DCMAKE_CXX_COMPILER=arm-linux-gnueabi-g++
make
```

For **ARM 64 bits toolchain**，In the Livox SDK2 directory，run the following commands to cross compile the project:

```
cd Livox-SDK
cd build && \
cmake .. -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
make
```

**Note:**

- gcc  cross compiler need to support C ++11 standard



### 2.2.1 Ubuntu 20.04/18.04/16.04LTS
For Ubuntun 20.04/18.04/16.04 LTS, run the *lidar_sample* if connect with the LiDAR unit(s):
```
cd sample/lidar && ./livox_lidar_quick_start ../../sample/livox_lidar_quick_start/config.json
```

for config.json file



### 2.2.2 Windows 7/10
~~After compiling the Livox SDK as shown in section 4.1.2, you can find `hub_sample.exe` or `lidar_sample.exe` in the {Livox-SDK}\build\sample\hub\Debug or {Livox-SDK}\build\sample\lidar\Debug folder, respectively, which can be run directly.~~ 

~~Then you can see the information as below:~~

~~![](doc/images/sdk_init.png)~~




### ~~2.4 Generate the lvx file~~

~~We provide the C++ sample to generate the lvx file for LiDAR unit(s). You can use the same way in `2.2.1` and `2.2.2` to run them.~~

~~需要在ROS-Driver2仓库下载ros驱动~~

~~（本SDK DRIVER单独不能使用lvx录制，需要结合ROS）~~

# 3 Support

You can get support from Livox with the following methods:
* Send email to cs@livoxtech.com with a clear description of your problem and your setup
* Github Issues

