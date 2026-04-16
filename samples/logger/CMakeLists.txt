CMAKE_MINIMUM_REQUIRED(VERSION 3.16.0 FATAL_ERROR)

set(DEMO_NAME logger)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static)

