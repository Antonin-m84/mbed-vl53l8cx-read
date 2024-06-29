# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET mbed-vl53l8cx-read)

include(${MBED_PATH}/tools/cmake/app.cmake)


add_subdirectory(${MBED_PATH})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drivers/vl53l8cx)

target_include_directories(ntp-client
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)
add_executable(${APP_TARGET})

target_compile_features(${APP_TARGET}
    PRIVATE cxx_std_17
)

mbed_set_mbed_target_linker_script(${APP_TARGET})

project(${APP_TARGET})

target_include_directories(${APP_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${APP_TARGET}
    PRIVATE
        src/main.cpp
        ${SRC_SOURCES}
)

target_link_libraries(${APP_TARGET}
    PRIVATE
        mbed-os
        vl53l8cx
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
