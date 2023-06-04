cmake_minimum_required(VERSION 3.0)

project(livox_sdk2)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

if (UNIX)
	set(CMAKE_CXX_STANDARD 20)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -std=gnu++0x")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=gnu++0x")
endif(UNIX)

add_subdirectory(sdk_core)
add_subdirectory(samples)
