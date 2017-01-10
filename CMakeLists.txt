cmake_minimum_required(VERSION 2.8)
project(EKF_location)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ekf_test ekf_main.cpp )
target_link_libraries( ekf_test ${OpenCV_LIBS} )
