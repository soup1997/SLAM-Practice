cmake_minimum_required(VERSION 3.0.0)
project(ekf_localization_cpp VERSION 0.1.0)

include(CTest)

find_package (Eigen3 3.3 NO_MODULE)
include_directories(include)
add_executable(ekf_localization_cpp main.cpp ekf.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
