cmake_minimum_required(VERSION 3.24)

set(DEMO_NAME debug_point_cloud)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static)

