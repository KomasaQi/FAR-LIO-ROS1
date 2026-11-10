cmake_minimum_required(VERSION 3.16)
project(farlio_odometry)

# ------------------------------------------------------------------------------
# FAR-LIO ROS 1 port
#
# Builds the unmodified FAR-LIO core library (lib_cpp/odometry_cpp, CPU build)
# together with a roscpp node. The upstream ROS 2 packages are NOT built; see
# ../../FAR-LIO (CATKIN_IGNORE) and docs/ROS1_PORT.md.
# ------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  farlio_msgs
)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Vendored Eigen 3.4.0: the FAR-LIO core uses Eigen::Vector<T,N> (3.4+), while
# Noetic's PCL 1.10 ships Eigen 3.3.7. Provided as the canonical Eigen3::Eigen
# target so the core uses it without a find_package(Eigen3) lookup.
set(FARLIO_EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen")
add_library(farlio_eigen INTERFACE)
# The upstream sources use the Debian-style include <eigen3/Eigen/Core>; the shim
# directory maps that onto the vendored 3.4 headers instead of /usr/include/eigen3.
target_include_directories(farlio_eigen INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen3-shim
  ${FARLIO_EIGEN_DIR}
)
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen ALIAS farlio_eigen)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs std_srvs
                 tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions farlio_msgs
  DEPENDS PCL TBB
)

# ------------------------------------------------------------------------------
# Vendored, ROS-independent TUM support libraries
# ------------------------------------------------------------------------------
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vendor/param_management_cpp ${CMAKE_CURRENT_BINARY_DIR}/vendor/param_management_cpp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vendor/tsl_logger_cpp ${CMAKE_CURRENT_BINARY_DIR}/vendor/tsl_logger_cpp)
# TUM types/helpers + the ROS-free state estimation library (headers)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vendor/state_estimation ${CMAKE_CURRENT_BINARY_DIR}/vendor/state_estimation)

# ------------------------------------------------------------------------------
# FAR-LIO core (upstream sources, CPU pipeline)
# ------------------------------------------------------------------------------
set(FAR_LIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../FAR-LIO" CACHE PATH "Path to the FAR-LIO checkout")
if(NOT EXISTS "${FAR_LIO_DIR}/lib_cpp/odometry_cpp/CMakeLists.txt")
  message(FATAL_ERROR "FAR-LIO core not found at ${FAR_LIO_DIR}")
endif()

set(BUILD_CUDA OFF CACHE BOOL "Build the CUDA pipeline" FORCE)
set(VISUALIZATION OFF CACHE BOOL "Build the rerun visualization" FORCE)
set(BUILD_ODOMETRY_EXAMPLES OFF CACHE BOOL "Build upstream examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build upstream unit tests" FORCE)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)

# Offline, reproducible dependencies (upstream would download these).
set(FETCHCONTENT_SOURCE_DIR_SOPHUS "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/Sophus" CACHE PATH "" FORCE)
set(FETCHCONTENT_SOURCE_DIR_ROBIN_MAP "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/robin-map" CACHE PATH "" FORCE)

add_subdirectory(${FAR_LIO_DIR}/lib_cpp/odometry_cpp ${CMAKE_CURRENT_BINARY_DIR}/odometry_cpp)

# Aggregate target so catkin can export a single "library".
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE odometry_pipeline)

# ------------------------------------------------------------------------------
# ROS 1 node
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/farlio_odometry_node.cpp
  src/gicp_cv_instantiation.cpp
)
add_executable(${PROJECT_NAME}_multi_lidar src/multi_lidar_merger.cpp)
add_executable(${PROJECT_NAME}_map_server src/map_server.cpp)
target_include_directories(${PROJECT_NAME}_multi_lidar PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost-shim
  ${catkin_INCLUDE_DIRS}
  ${FARLIO_EIGEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen3-shim
)
target_link_libraries(${PROJECT_NAME}_multi_lidar PRIVATE ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_multi_lidar ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}_map_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost-shim
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen3-shim
  ${FARLIO_EIGEN_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_map_server PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} usb-1.0)
add_dependencies(${PROJECT_NAME}_map_server ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost-shim
  ${FARLIO_EIGEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen3-shim
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
  odometry_pipeline
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  usb-1.0
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------
# State estimation node (5 variants: 2D/3D EKF x Kinematic/NonHolonomic/SingleTrack)
# ------------------------------------------------------------------------------
set(FARLIO_ESTIMATION_VARIANTS 2d_kin 2d_nh 3d_kin 3d_nh 3d_stm)
set(FARLIO_ESTIMATION_TARGETS "")
foreach(variant ${FARLIO_ESTIMATION_VARIANTS})
  set(target ${PROJECT_NAME}_state_estimation_${variant})
  add_executable(${target} src/state_estimation_${variant}.cpp)
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost-shim
    ${FARLIO_EIGEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen3-shim
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${target} PRIVATE
    tam_state_estimation
    ${catkin_LIBRARIES}
    yaml-cpp
    Boost::system
  )
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
  list(APPEND FARLIO_ESTIMATION_TARGETS ${target})
endforeach()

install(TARGETS ${FARLIO_ESTIMATION_TARGETS}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_multi_lidar ${PROJECT_NAME}_map_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
