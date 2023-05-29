cmake_minimum_required(VERSION 3.0.0)
project(ekf_slam_cpp VERSION 0.1.0)

find_package (Eigen3 3.3 NO_MODULE)
include_directories(include ekf util)

include(CTest)
enable_testing()

add_executable(${PROJECT_NAME} src/main.cpp src/ekf_slam.cpp src/tools.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
