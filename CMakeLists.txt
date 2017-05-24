cmake_minimum_required(VERSION 2.8.12)

add_subdirectory(dlib)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

option(LIGHTWEIGHT "compile w/o dlib" ON)

rosbuild_init()

find_package(OpenCV 3.1 REQUIRED)

#rosbuild_genmsg()


#compile detector
rosbuild_add_executable(object_tracker src/tracker.cpp)
rosbuild_add_compile_flags(object_tracker -std=c++11)
TARGET_LINK_LIBRARIES(object_tracker ${OpenCV_LIBS} dlib::dlib)


