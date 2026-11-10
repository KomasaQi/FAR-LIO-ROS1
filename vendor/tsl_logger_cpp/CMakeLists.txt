# ROS 1 vendored copy of TUMFTM/tsl :: tsl_logger_cpp (header-only, plain CMake)
cmake_minimum_required(VERSION 3.16)
project(tsl_logger_cpp)

add_library(tsl_logger_cpp INTERFACE)
target_include_directories(tsl_logger_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(tsl_logger_cpp INTERFACE cxx_std_17)
add_library(tsl_logger_cpp::tsl_logger_cpp ALIAS tsl_logger_cpp)
