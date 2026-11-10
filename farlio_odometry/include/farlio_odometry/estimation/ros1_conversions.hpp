// Licensed under the Apache License, Version 2.0.
//
// ROS 1 port of TUMFTM/TAM__common :: tum_type_conversions_ros_cpp.
//
// The conversion functions keep the upstream names and signatures (only the
// ROS message types change from ROS 2 to ROS 1) so the ported state-estimation
// node logic stays identical to upstream.
#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <farlio_msgs/TUMFloat64PerWheel.h>

#include "tum_helpers_cpp/type_conversion.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace tam::type_conversions {
// ---------------------------------------------------------------------------
// Orientation
// ---------------------------------------------------------------------------
inline tam::types::common::EulerYPR quaternion_msg_to_euler_type(
  const geometry_msgs::Quaternion& msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  tf2::Matrix3x3 m(q);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  m.getEulerYPR(yaw, pitch, roll);
  return tam::types::common::EulerYPR(yaw, pitch, roll);
}

inline geometry_msgs::Quaternion euler_type_to_quaternion_msg(
  const tam::types::common::EulerYPR& euler_angles)
{
  tf2::Quaternion q;
  q.setEulerZYX(euler_angles.yaw, euler_angles.pitch, euler_angles.roll);
  q.normalize();
  geometry_msgs::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

// ---------------------------------------------------------------------------
// Basic types
// ---------------------------------------------------------------------------
inline tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  const geometry_msgs::Vector3& msg)
{
  return {msg.x, msg.y, msg.z};
}

inline tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  const geometry_msgs::Point& msg)
{
  return {msg.x, msg.y, msg.z};
}

inline geometry_msgs::Vector3 vector_3d_msg_from_type(
  const tam::types::common::Vector3D<double>& vector_3d)
{
  geometry_msgs::Vector3 msg;
  msg.x = vector_3d.x;
  msg.y = vector_3d.y;
  msg.z = vector_3d.z;
  return msg;
}

inline geometry_msgs::Point point_msg_from_type(
  const tam::types::common::Vector3D<double>& vector_3d)
{
  geometry_msgs::Point msg;
  msg.x = vector_3d.x;
  msg.y = vector_3d.y;
  msg.z = vector_3d.z;
  return msg;
}

inline tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  const farlio_msgs::TUMFloat64PerWheel& msg)
{
  return tam::types::common::DataPerWheel<double>{
    msg.front_left, msg.front_right, msg.rear_left, msg.rear_right};
}

// ---------------------------------------------------------------------------
// State estimation conversions
// ---------------------------------------------------------------------------
inline tam::types::control::Odometry odometry_type_from_msg(const nav_msgs::Odometry& msg)
{
  tam::types::control::Odometry odometry;
  odometry.position_m.x = msg.pose.pose.position.x;
  odometry.position_m.y = msg.pose.pose.position.y;
  odometry.position_m.z = msg.pose.pose.position.z;
  odometry.orientation_rad = tam::helpers::types::to_vector3d(
    tam::type_conversions::quaternion_msg_to_euler_type(msg.pose.pose.orientation));
  std::copy(msg.pose.covariance.begin(), msg.pose.covariance.end(), odometry.pose_covariance.begin());
  odometry.angular_velocity_radps.x = msg.twist.twist.angular.x;
  odometry.angular_velocity_radps.y = msg.twist.twist.angular.y;
  odometry.angular_velocity_radps.z = msg.twist.twist.angular.z;
  odometry.velocity_mps.x = msg.twist.twist.linear.x;
  odometry.velocity_mps.y = msg.twist.twist.linear.y;
  odometry.velocity_mps.z = msg.twist.twist.linear.z;
  std::copy(
    msg.twist.covariance.begin(), msg.twist.covariance.end(),
    odometry.velocity_covariance.begin());
  return odometry;
}

inline nav_msgs::Odometry odometry_msg_from_type(const tam::types::control::Odometry& odometry)
{
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = odometry.position_m.x;
  msg.pose.pose.position.y = odometry.position_m.y;
  msg.pose.pose.position.z = odometry.position_m.z;
  msg.pose.pose.orientation = tam::type_conversions::euler_type_to_quaternion_msg(
    tam::types::common::EulerYPR(
      odometry.orientation_rad.z, odometry.orientation_rad.y, odometry.orientation_rad.x));
  std::copy(
    odometry.pose_covariance.begin(), odometry.pose_covariance.end(), msg.pose.covariance.begin());
  msg.twist.twist.linear.x = odometry.velocity_mps.x;
  msg.twist.twist.linear.y = odometry.velocity_mps.y;
  msg.twist.twist.linear.z = odometry.velocity_mps.z;
  msg.twist.twist.angular.x = odometry.angular_velocity_radps.x;
  msg.twist.twist.angular.y = odometry.angular_velocity_radps.y;
  msg.twist.twist.angular.z = odometry.angular_velocity_radps.z;
  std::copy(
    odometry.velocity_covariance.begin(), odometry.velocity_covariance.end(),
    msg.twist.covariance.begin());
  return msg;
}

inline tam::types::control::Odometry odometry_type_from_imu_msg(const sensor_msgs::Imu& msg)
{
  tam::types::control::Odometry odometry;
  odometry.angular_velocity_radps.x = msg.angular_velocity.x;
  odometry.angular_velocity_radps.y = msg.angular_velocity.y;
  odometry.angular_velocity_radps.z = msg.angular_velocity.z;
  for (std::size_t i = 0; i < 3; ++i) {
    std::copy_n(
      msg.angular_velocity_covariance.begin() + i * 3, 3,
      odometry.velocity_covariance.begin() + (i + 3) * 6 + 3);
  }
  return odometry;
}

inline tam::types::control::AccelerationwithCovariances
acceleration_with_covariances_type_from_imu_msg(const sensor_msgs::Imu& msg)
{
  tam::types::control::AccelerationwithCovariances acceleration;
  acceleration.acceleration_mps2.x = msg.linear_acceleration.x;
  acceleration.acceleration_mps2.y = msg.linear_acceleration.y;
  acceleration.acceleration_mps2.z = msg.linear_acceleration.z;
  for (std::size_t i = 0; i < 3; ++i) {
    std::copy_n(
      msg.linear_acceleration_covariance.begin() + i * 3, 3,
      acceleration.acceleration_covariance.begin() + i * 6);
  }
  return acceleration;
}

inline geometry_msgs::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances& acceleration)
{
  geometry_msgs::AccelWithCovarianceStamped msg;
  msg.accel.accel.linear = tam::type_conversions::vector_3d_msg_from_type(
    acceleration.acceleration_mps2);
  msg.accel.accel.angular = tam::type_conversions::vector_3d_msg_from_type(
    acceleration.angular_acceleration_radps2);
  std::copy(
    acceleration.acceleration_covariance.begin(), acceleration.acceleration_covariance.end(),
    msg.accel.covariance.begin());
  return msg;
}

inline tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  const geometry_msgs::AccelWithCovarianceStamped& msg)
{
  tam::types::control::AccelerationwithCovariances type;
  type.acceleration_mps2 = tam::type_conversions::vector_3d_type_from_msg(msg.accel.accel.linear);
  type.angular_acceleration_radps2 = tam::type_conversions::vector_3d_type_from_msg(
    msg.accel.accel.angular);
  std::copy(
    msg.accel.covariance.begin(), msg.accel.covariance.end(),
    type.acceleration_covariance.begin());
  return type;
}
}  // namespace tam::type_conversions

// Backwards-compatible aliases used by some TUM headers (same as upstream).
namespace tam::types::conversion {
using tam::type_conversions::euler_type_to_quaternion_msg;
using tam::type_conversions::quaternion_msg_to_euler_type;
}  // namespace tam::types::conversion
