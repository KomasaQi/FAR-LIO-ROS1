cmake_minimum_required(VERSION 3.16)
project(farlio_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    TUMFloat64PerWheel.msg
    TUMFloat64PerWheelStamped.msg
    SteeringReport.msg
    TSLValues.msg
    TSLDefinition.msg
)

add_service_files(
  DIRECTORY srv
  FILES GetPointCloudMap.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
