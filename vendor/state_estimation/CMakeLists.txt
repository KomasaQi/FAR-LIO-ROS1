# Vendored, ROS-independent TUM libraries used by the FAR-LIO state estimation.
# Built as part of farlio_state_estimation (plain CMake, no ament).

# ------------------------------------------------------------------------------
# tum_types_cpp (header only)
# ------------------------------------------------------------------------------
add_library(tum_types_cpp INTERFACE)
target_include_directories(tum_types_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tum_types_cpp/include>
)
target_compile_features(tum_types_cpp INTERFACE cxx_std_20)
target_link_libraries(tum_types_cpp INTERFACE tsl_logger_cpp::tsl_logger_cpp)
add_library(tum_types_cpp::tum_types_cpp ALIAS tum_types_cpp)

# ------------------------------------------------------------------------------
# tum_helpers_cpp
# ------------------------------------------------------------------------------
set(TUM_HELPERS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../tum_helpers_cpp/src)
add_library(tum_helpers_cpp SHARED
  ${TUM_HELPERS_SRC}/delay_compensation.cpp
  ${TUM_HELPERS_SRC}/discrete_filters/fir.cpp
)
target_include_directories(tum_helpers_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tum_helpers_cpp/include>
)
target_compile_features(tum_helpers_cpp PUBLIC cxx_std_20)
target_link_libraries(tum_helpers_cpp PUBLIC tum_types_cpp Eigen3::Eigen Boost::boost Boost::system)
set_target_properties(tum_helpers_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(tum_helpers_cpp::tum_helpers_cpp ALIAS tum_helpers_cpp)

# ------------------------------------------------------------------------------
# tum_type_conversions_ros_cpp (ROS 1 shim)
# ------------------------------------------------------------------------------
add_library(tum_type_conversions_ros_cpp INTERFACE)
target_include_directories(tum_type_conversions_ros_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tum_type_conversions_ros_cpp/include>
)
target_link_libraries(tum_type_conversions_ros_cpp INTERFACE tum_types_cpp)
add_library(tum_type_conversions_ros_cpp::tum_type_conversions_ros_cpp ALIAS tum_type_conversions_ros_cpp)

# ------------------------------------------------------------------------------
# state_estimation_constants / state_estimation_base / tam_state_estimation
# ------------------------------------------------------------------------------
add_library(state_estimation_constants INTERFACE)
target_include_directories(state_estimation_constants INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/state_estimation_constants>
)
target_link_libraries(state_estimation_constants INTERFACE Eigen3::Eigen tum_types_cpp)
target_compile_features(state_estimation_constants INTERFACE cxx_std_20)
add_library(state_estimation_constants::state_estimation_constants ALIAS state_estimation_constants)

add_library(state_estimation_base INTERFACE)
target_include_directories(state_estimation_base INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/state_estimation_base>
)
target_link_libraries(state_estimation_base INTERFACE
  Eigen3::Eigen tum_types_cpp param_management_cpp::param_management_cpp
  tsl_logger_cpp::tsl_logger_cpp state_estimation_constants)
target_compile_features(state_estimation_base INTERFACE cxx_std_20)
add_library(state_estimation_base::state_estimation_base ALIAS state_estimation_base)

add_library(tam_state_estimation INTERFACE)
target_include_directories(tam_state_estimation INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tam_state_estimation>
)
target_link_libraries(tam_state_estimation INTERFACE
  Eigen3::Eigen
  tum_types_cpp
  tum_helpers_cpp
  tum_type_conversions_ros_cpp
  param_management_cpp::param_management_cpp
  tsl_logger_cpp::tsl_logger_cpp
  state_estimation_constants
  state_estimation_base
)
target_compile_features(tam_state_estimation INTERFACE cxx_std_20)
add_library(tam_state_estimation::tam_state_estimation ALIAS tam_state_estimation)
