project(EKF)

cmake_minimum_required(VERSION 3.18.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)

add_executable(EKF ${SOURCE_FILES})
target_link_libraries(EKF Eigen3::Eigen)
