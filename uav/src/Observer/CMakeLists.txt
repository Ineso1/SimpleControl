PROJECT(Observer)
cmake_minimum_required(VERSION 2.8)

include_directories("/usr/include/eigen3")
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
add_compile_options(-Wno-poison-system-directories)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/UDE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/SlidingMode)

