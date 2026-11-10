// Copyright 2025 Maximilian Leitenstern

#pragma once
// CAUTION: These headers are not included as dependencies in
// tum_types_cpp/CMakeLists.txt, as this would require them to be in
// the base image.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
namespace tam::types::drivers
{
template <typename InterfaceType, typename PointType>
struct PointCloudSensorState
{
  std::shared_ptr<InterfaceType> data_interface;
  pcl::PointCloud<PointType>::Ptr last_received_pcl;
  bool has_received_data{false};
  std::string frame_id;
  geometry_msgs::msg::TransformStamped transform;
  int64_t first_timestamp;
  int64_t last_timestamp;
  bool publishSingle;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr singlePublisher;
};
}  // namespace tam::types::drivers
