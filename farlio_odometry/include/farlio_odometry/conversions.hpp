// Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ROS 1 port of tam_odometry/include/tam_odometry/utils.hpp (FAR-LIO).
// Only message plumbing changed (rclcpp -> roscpp); the conversion semantics are
// identical, plus an additional "offset_time" candidate for Livox drivers.
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sophus/se3.hpp>
#include <optional>
#include <std_msgs/Header.h>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
#include <ros/time.h>

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/odometry_types.hpp"
#include "odometry_types/point_types.hpp"

// Macros for optional PointCloud2 fields handled by cloud2eigen / eigen2cloud.
// F(member, c++-type, ros_datatype_suffix, write_name, read_name_aliases...)
#define SPHERICAL_FIELDS(F)                        \
  F(azimuth, float, FLOAT32, "azimuth", "azimuth") \
  F(range, float, FLOAT32, "range", "range")       \
  F(elevation, float, FLOAT32, "elevation", "elevation")
#define INTENSITY_FIELDS(F) F(intensity, float, FLOAT32, "intensity", "intensity")
#define RADAR_FIELDS(F)                                     \
  F(vel, float, FLOAT32, "velocity", "velocity")            \
  F(rcs, float, FLOAT32, "rcs", "rcs")                      \
  F(snr, float, FLOAT32, "snr", "snr")                      \
  F(confidence, float, FLOAT32, "confidence", "confidence") \
  F(vel_interval, float, FLOAT32, "vel_interval", "velocity_interval")
#define SENSOR_ID_FIELDS(F) F(sensor_id, std::uint8_t, UINT8, "sensor_id", "sensor_id", "id")

// Read Macros
#define DECL_CONST_ITER(member, type, dtype, wname, ...) \
  std::optional<sensor_msgs::PointCloud2ConstIterator<type>> it_##member;
#define INIT_CONST_ITER(member, type, dtype, wname, ...)                       \
  it_##member.emplace(                                                         \
    msg, farlio_ros1::conversions::find_field(msg, sensor_msgs::PointField::dtype, __VA_ARGS__));
#define CHECK_FIELD(member, type, dtype, wname, ...) \
  || farlio_ros1::conversions::find_field(msg, sensor_msgs::PointField::dtype, __VA_ARGS__).empty()
#define READ_ITER(member, type, dtype, wname, ...) \
  points[i].member = **it_##member;                \
  ++(*it_##member);

// Write Macros
#define DECL_ITER(member, type, dtype, wname, ...) \
  std::optional<sensor_msgs::PointCloud2Iterator<type>> it_##member;
#define INIT_ITER(member, type, dtype, wname, ...) it_##member.emplace(cloud_msg, wname);
#define WRITE_ITER(member, type, dtype, wname, ...) \
  **it_##member = points[i].member;                 \
  ++(*it_##member);
#define ADD_FIELD(member, type, dtype, wname, ...) \
  offset = addPointField(cloud_msg, wname, 1, sensor_msgs::PointField::dtype, offset);

namespace farlio_ros1 {
namespace conversions {
/**
 * @brief Convert a ROS 1 time stamp to nanoseconds
 */
inline std::uint64_t stamp2ns(const ros::Time& stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(stamp.nsec);
}

/**
 * @brief Sophus pose -> geometry_msgs::Pose
 */
inline geometry_msgs::Pose sophus2pose(const Sophus::SE3f& T)
{
  geometry_msgs::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  Eigen::Quaternionf q(T.so3().unit_quaternion());
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();
  return t;
}

/**
 * @brief Sophus pose -> geometry_msgs::Transform
 */
inline geometry_msgs::Transform sophus2transform(const Sophus::SE3f& T)
{
  geometry_msgs::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  Eigen::Quaternionf q(T.so3().unit_quaternion());
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  return t;
}

/**
 * @brief tf2 transform -> internal pose
 */
inline tam::core::state::types::PoseStamped transform2pose(
  const geometry_msgs::TransformStamped& transform)
{
  tam::core::state::types::PoseStamped pose;
  pose.stamp = stamp2ns(transform.header.stamp);
  const auto& t = transform.transform;
  pose.pose = Sophus::SE3f(
    Sophus::SE3f::QuaternionType(
      t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
    Sophus::SE3f::Point(t.translation.x, t.translation.y, t.translation.z));
  return pose;
}

/**
 * @brief internal odometry -> ROS 1 nav_msgs::Odometry
 */
inline nav_msgs::Odometry odom2msg(
  const tam::core::state::types::Odometry& odom, const std::string& frame_id,
  const std::string& child_frame_id)
{
  nav_msgs::Odometry msg;
  msg.header.stamp.sec = static_cast<uint32_t>(odom.stamp / 1000000000ULL);
  msg.header.stamp.nsec = static_cast<uint32_t>(odom.stamp % 1000000000ULL);
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.pose.pose = sophus2pose(odom.pose.pose);
  msg.twist.twist.linear.x = odom.tangent.tangent[0];
  msg.twist.twist.linear.y = odom.tangent.tangent[1];
  msg.twist.twist.linear.z = odom.tangent.tangent[2];
  msg.twist.twist.angular.x = odom.tangent.tangent[3];
  msg.twist.twist.angular.y = odom.tangent.tangent[4];
  msg.twist.twist.angular.z = odom.tangent.tangent[5];
  std::transform(
    odom.pose.covariance.begin(), odom.pose.covariance.end(), msg.pose.covariance.begin(),
    [](float val) { return static_cast<double>(val); });
  std::transform(
    odom.tangent.covariance.begin(), odom.tangent.covariance.end(), msg.twist.covariance.begin(),
    [](float val) { return static_cast<double>(val); });
  return msg;
}

/**
 * @brief Load a static point cloud map from a .pcd file
 */
template <typename TConfig>
std::vector<tam::core::state::types::Point<TConfig>> load_static_map(const std::string& file_path)
{
  std::cout << "Load map from " << file_path << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<tam::core::state::types::Point<TConfig>> map{};

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
    std::cout << "Couldn't read file " << file_path << std::endl;
    return map;
  }
  map.reserve(cloud->size());
  for (const auto& pt : cloud->points) {
    tam::core::state::types::Point<TConfig> p;
    p.pos = Eigen::Vector3f{pt.x, pt.y, pt.z};
    if constexpr (TConfig::INTENSITY) p.intensity = pt.intensity;
    map.emplace_back(p);
  }
  std::cout << "Loading completed: " << cloud->width * cloud->height << " points" << std::endl;
  return map;
}

/**
 * @brief Search for a field in a PointCloud2 msg, given a list of candidates
 */
template <typename... S>
inline std::string find_field(
  const sensor_msgs::PointCloud2& msg, std::uint8_t datatype, S&&... candidates)
{
  for (const auto& f : msg.fields) {
    if (f.datatype != datatype) continue;
    if (((f.name == candidates) || ...)) return f.name;
  }
  return {};
}

/**
 * @brief transform incoming pointcloud to target frame and convert it to a point vector
 *
 * @param[in] msg                - sensor_msgs::PointCloud2 incoming frame
 * @param[in] iso                - Eigen::Isometry3f transformation to apply
 */
template <typename TConfig>
std::vector<tam::core::state::types::Point<TConfig>> cloud2eigen(
  const sensor_msgs::PointCloud2& msg, const Eigen::Isometry3f& iso)
{
  const double msg_time = static_cast<double>(msg.header.stamp.toSec());
  const size_t num_points = static_cast<size_t>(msg.height) * msg.width;
  std::vector<tam::core::state::types::Point<TConfig>> points(num_points);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

  SPHERICAL_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::SPHERICAL) {
    if (false SPHERICAL_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error(
        "[farlio_odometry]: Missing 'azimuth'/'range'/'elevation' fields in PointCloud2 message");
    }
    SPHERICAL_FIELDS(INIT_CONST_ITER)
  }
  INTENSITY_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::INTENSITY) {
    if (false INTENSITY_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[farlio_odometry]: Missing 'intensity' field in PointCloud2 message");
    }
    INTENSITY_FIELDS(INIT_CONST_ITER)
  }

  RADAR_FIELDS(DECL_CONST_ITER)
  if constexpr (tam::core::state::types::HASRADAR<TConfig>) {
    if (false RADAR_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[farlio_odometry]: Missing radar fields in PointCloud2 message");
    }
    RADAR_FIELDS(INIT_CONST_ITER)
  }

  SENSOR_ID_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::SENSOR_ID) {
    if (false SENSOR_ID_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[farlio_odometry]: Missing 'sensor_id' field in PointCloud2 message");
    }
    SENSOR_ID_FIELDS(INIT_CONST_ITER)
  }

  // Timestamp: optional + variant since the field name and datatype are runtime-determined.
  std::optional<std::variant<sensor_msgs::PointCloud2ConstIterator<uint32_t>,
                             sensor_msgs::PointCloud2ConstIterator<float>,
                             sensor_msgs::PointCloud2ConstIterator<double>>>
    msg_ts;
  for (const auto& field : msg.fields) {
    // "offset_time" is used by Livox drivers (uint32 ns offset from header stamp).
    if (
      field.name == "t" || field.name == "timestamp" || field.name == "time" ||
      field.name == "offset_time") {
      if (field.datatype == sensor_msgs::PointField::UINT32) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<uint32_t>(msg, field.name));
      } else if (field.datatype == sensor_msgs::PointField::FLOAT32) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<float>(msg, field.name));
      } else if (field.datatype == sensor_msgs::PointField::FLOAT64) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<double>(msg, field.name));
      }
      break;
    }
  }

  for (size_t i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z) {
    auto& pt = points[i];
    pt.pos = iso * Eigen::Vector3f{*it_x, *it_y, *it_z};

    if constexpr (TConfig::SPHERICAL) {
      SPHERICAL_FIELDS(READ_ITER)
    }
    if constexpr (TConfig::INTENSITY) {
      INTENSITY_FIELDS(READ_ITER)
    }
    if constexpr (tam::core::state::types::HASRADAR<TConfig>) {
      RADAR_FIELDS(READ_ITER)
    }
    if constexpr (TConfig::SENSOR_ID) {
      SENSOR_ID_FIELDS(READ_ITER)
    }

    if (msg_ts) {
      std::visit(
        [&](auto& it) {
          double ts_s = 0.0;
          auto raw = *it;
          ++it;
          using raw_t = std::decay_t<decltype(raw)>;
          if constexpr (std::is_same_v<raw_t, uint32_t>) {
            // nanosecond offset from the message header stamp (Ouster / Livox)
            ts_s = static_cast<double>(raw) * 1e-9;
          } else if constexpr (std::is_same_v<raw_t, float>) {
            // seconds offset from the message header stamp (Velodyne)
            ts_s = static_cast<double>(raw);
          } else {
            // absolute timestamp in seconds (Hesai / TAM)
            ts_s = static_cast<double>(raw) - msg_time;
          }
          pt.timestamp = static_cast<float>(ts_s);
        },
        *msg_ts);
    }
  }
  return points;
}

/**
 * @brief transform a point vector to a sensor_msgs::PointCloud2 message
 */
template <typename TConfig>
sensor_msgs::PointCloud2 eigen2cloud(
  const std::vector<tam::core::state::types::Point<TConfig>>& points,
  const std_msgs::Header& header)
{
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header = header;
  cloud_msg.height = 1;
  cloud_msg.width = static_cast<uint32_t>(points.size());
  cloud_msg.is_dense = false;
  cloud_msg.is_bigendian = false;

  int offset = 0;
  offset = addPointField(cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  if constexpr (TConfig::INTENSITY) {
    ADD_FIELD(intensity, float, FLOAT32, "intensity")
  }
  cloud_msg.point_step = static_cast<uint32_t>(offset);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step) * cloud_msg.height);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");
  DECL_ITER(intensity, float, FLOAT32, "intensity")
  if constexpr (TConfig::INTENSITY) INIT_ITER(intensity, float, FLOAT32, "intensity")

  for (size_t i = 0; i < points.size(); ++i, ++it_x, ++it_y, ++it_z) {
    *it_x = points[i].pos.x();
    *it_y = points[i].pos.y();
    *it_z = points[i].pos.z();
    if constexpr (TConfig::INTENSITY) WRITE_ITER(intensity, float, FLOAT32, "intensity")
  }
  return cloud_msg;
}
}  // namespace conversions
}  // namespace farlio_ros1
