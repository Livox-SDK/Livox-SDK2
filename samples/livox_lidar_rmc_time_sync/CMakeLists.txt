CMAKE_MINIMUM_REQUIRED(VERSION 3.16.0 FATAL_ERROR)

set(DEMO_NAME livox_lidar_rmc_time_sync)
if (WIN32)
  add_executable(${DEMO_NAME} main.cpp win/synchro.cpp)
elseif(UNIX)
  add_executable(${DEMO_NAME} main.cpp linux/synchro.cpp)
endif()

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static
)
