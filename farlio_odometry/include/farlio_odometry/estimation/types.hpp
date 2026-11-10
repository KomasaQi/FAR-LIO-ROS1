// Licensed under the Apache License, Version 2.0.
//
// ROS 1 port of tam_state_estimation_node/types.hpp.
#pragma once

#include <ros/time.h>
#include <tf2/LinearMath/Vector3.h>

#include <cstdint>
#include <string>
#include <vector>

#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace tam::types::state::config {
struct node {
  std::string odom_frame_id{};
  std::string child_frame_id{};
  double virtual_covariance_scale{};
  int32_t average_input_delay_ms{};
};

struct subscription {
  std::string topic{};
  std::string type{};
  bool active{false};
  int32_t timeout_ms{};
};

struct fusion {
  std::vector<double> R_init{};
  std::vector<double> outlier_bounds{};
  std::vector<bool> mask{};
  uint8_t num{};
  bool active{false};
};

struct measurement {
  std::string name{};
  tam::types::state::config::subscription measurement_sub{};
  tam::types::state::config::subscription status_sub{};
  tam::types::state::config::fusion position_m{};
  tam::types::state::config::fusion orientation_rad{};
  tam::types::state::config::fusion linear_velocity_mps{};
  int32_t additional_delay{};
};

struct input {
  std::string name{};
  tam::types::state::config::subscription measurement_sub{};
  std::vector<double> filter_coefficients;
  uint8_t num{};
  bool backup{false};
};
}  // namespace tam::types::state::config

namespace tam::types::state {
struct measurement {
  tam::types::state::config::measurement config{};
  tam::types::control::Odometry odometry{};
  ros::Time measurement_stamp{};
  ros::Time received_stamp{};
  tf2::Vector3 static_translation{};
  bool static_translation_valid{false};
  bool received{false};
};

struct input {
  tam::types::state::config::input config{};
  bool received{false};
};
}  // namespace tam::types::state
