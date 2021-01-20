cmake_minimum_required(VERSION 3.0)
project (LYCV)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Debug)

#set(src_file
#main.cpp
##HikvisionCamera.cpp
##infrared_solvePnP.cpp
##serial/src/serial.cc
##serial/src/impl/unix.cc
##serial/src/impl/win.cc
##serial/src/impl/list_ports/list_ports_linux.cc
##serial/src/impl/list_ports/list_ports_osx.cc
##serial/src/impl/list_ports/list_ports_win.cc
#)

#寻找OpenCV库
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#include_directories(
##"serial/include"
##"/opt/MVS/include"
#)

#link_directories(
##"/opt/MVS/lib/aarch64/"
#)

add_executable(LYCV main.cpp infrared_solvePnP.cpp infrared_solvePnP.h)
#target_link_libraries(LYCV MvCameraControl opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc opencv_calib3d opencv_objdetect pthread)
target_link_libraries(LYCV ${OpenCV_LIBS})


