PROJECT(SimpleControl)
cmake_minimum_required(VERSION 2.8)
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/simulator)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/uav)
