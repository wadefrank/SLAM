cmake_minimum_required( VERSION 3.10 )
project( useEigen )

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(Eigen3_DIR /home/wade/third_party/eigen-3.3.9/install/share/eigen3/cmake)


# Eigen3
find_package(Eigen3 3.3 REQUIRED)
message(STATUS "Eigen include directory: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(useEigen src/useEigen.cpp)
target_link_libraries(useEigen)