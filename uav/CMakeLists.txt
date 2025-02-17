PROJECT(SimpleControl)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
include_directories("/usr/include/eigen3")

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

add_subdirectory(src/Control)
add_subdirectory(src/Observer)

SET(SRC_FILES
	src/main.cpp
    src/Control/Drone.cpp
    src/Control/DroneBase.cpp
    src/Control/Law.cpp
    src/Control/Trayectory.cpp
    src/Control/SoftTrajectoryGenerator.cpp
    src/Observer/ObserverBase.cpp
    src/Observer/UDE/UDE.cpp
    src/Observer/SlidingMode/SlidingMode.cpp
    src/Observer/SuperTwist/SuperTwist.cpp
    src/Observer/Luenberger/Luenberger.cpp
    src/Observer/KFC/KFC.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR SimpleControl
           COPY_RESOURCES
)
