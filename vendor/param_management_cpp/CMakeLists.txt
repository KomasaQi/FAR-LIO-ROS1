# ROS 1 vendored copy of TUMFTM/TAM__common :: param_management_cpp
# (plain CMake, no ament; built as part of farlio_odometry)
cmake_minimum_required(VERSION 3.16)
project(param_management_cpp)

add_library(param_management_cpp SHARED
  src/init_backend.cpp
  src/initialization.cpp
  src/param_value_manager.cpp
  src/param_reference_manager.cpp
  src/param_manager_composer.cpp
)
target_include_directories(param_management_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(param_management_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(param_management_cpp::param_management_cpp ALIAS param_management_cpp)

# The upstream headers are C++17; the FAR-LIO core requires C++20. Make the
# requirement explicit for consumers.
target_compile_features(param_management_cpp PUBLIC cxx_std_17)
