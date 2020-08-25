cmake_minimum_required(VERSION 3.18.1)
project(EKF-sensor-fusion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)

set(SOURCE_FILES src/main.cc src/ekf.cc src/tools.cc)
add_executable(EKF ${SOURCE_FILES})
target_include_directories(EKF PRIVATE include)
target_link_libraries(EKF Eigen3::Eigen)
