// Licensed under the Apache License, Version 2.0.
//
// ROS 1 port of tam_state_estimation_node/helper.hpp.
//
// Upstream enumerates the ROS 2 parameter overrides to discover the declarative
// sensor configuration (measurements.* / inputs.imus.*). On ROS 1 the same keys
// are read from the YAML configuration file (see yaml_params.hpp) and passed in
// as typed maps; the configuration semantics are unchanged.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "farlio_odometry/yaml_params.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "state_estimation_base/types.hpp"
#include "types.hpp"

namespace tam::core::state {

inline bool is_odometry_type(const std::string& type)
{
  return type == "nav_msgs/msg/Odometry" || type == "nav_msgs/Odometry";
}

inline bool is_imu_type(const std::string& type)
{
  return type == "sensor_msgs/msg/Imu" || type == "sensor_msgs/Imu";
}

/**
 * @brief Parse the measurement / input sensor configuration from the YAML config
 */
template <class TConfig>
void parse_sensor_configs(
  tam::pmg::MgmtInterface* param_manager_raw,
  const std::map<std::string, farlio_ros1::ParamMap>& measurement_map,
  const std::map<std::string, farlio_ros1::ParamMap>& input_map,
  std::vector<tam::types::state::measurement>& measurement_channels,
  std::vector<tam::types::state::input>& input_channels,
  std::vector<std::pair<tam::types::state::measurements::identifier, std::string>>&
    identifier_names)
{
  // Match config to Measurement Configuration Data Structure
  uint8_t num_position{}, num_orientation{}, num_linear_velocity{};
  for (const auto& [name, parameter] : measurement_map) {
    tam::types::state::measurement channel{};
    tam::types::state::config::measurement& cfg = channel.config;
    cfg.name = name;

    if (!parameter.count("message.topic"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
        " does not provide a topic");
    if (!parameter.count("message.type"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
        " does not provide a topic type");

    cfg.measurement_sub.topic = parameter.at("message.topic").template get<std::string>();
    cfg.measurement_sub.type = parameter.at("message.type").template get<std::string>();
    cfg.measurement_sub.timeout_ms =
      parameter.count("message.timeout_ms") ? parameter.at("message.timeout_ms").template get<std::int32_t>()
                                            : 500;
    cfg.measurement_sub.active = true;

    if (!is_odometry_type(cfg.measurement_sub.type))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
        " has an unimplemented topic type '" + cfg.measurement_sub.type +
        "', only 'nav_msgs/Odometry' (or 'nav_msgs/msg/Odometry') is supported");

    if (parameter.count("status.topic")) {
      cfg.status_sub.topic = parameter.at("status.topic").template get<std::string>();
      cfg.status_sub.timeout_ms =
        parameter.count("status.timeout_ms") ? parameter.at("status.timeout_ms").template get<std::int32_t>()
                                             : 500;
      cfg.status_sub.active = true;
    } else {
      std::cout << "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
                     " does not provide a status topic, input is used without status"
                << std::endl;
      cfg.status_sub.active = false;
    }

    cfg.additional_delay =
      parameter.count("message.additional_delay_ms")
        ? parameter.at("message.additional_delay_ms").template get<std::int32_t>()
        : 0;

    auto setup_fusion_config =
      [&](const std::string& prefix, tam::types::state::config::fusion& fusion, uint8_t& num) {
        if (!parameter.count(prefix + ".R_init") || !parameter.count(prefix + ".outlier_bounds"))
          return;
        fusion.R_init = parameter.at(prefix + ".R_init").template get<std::vector<double>>();
        fusion.outlier_bounds = parameter.at(prefix + ".outlier_bounds").template get<std::vector<double>>();
        for (const double r : fusion.R_init) {
          if (!std::isfinite(r) || r <= 0.0)
            throw std::invalid_argument(
              "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
              " has a non-positive R_init entry for " + prefix);
        }
        fusion.mask = parameter.count(prefix + ".mask")
                        ? parameter.at(prefix + ".mask").template get<std::vector<bool>>()
                        : std::vector<bool>(fusion.R_init.size(), true);
        fusion.num = num++;
        fusion.active = true;
        if (
          fusion.R_init.size() != fusion.outlier_bounds.size() ||
          fusion.R_init.size() != fusion.mask.size())
          throw std::invalid_argument(
            "[StateEstimationCPP]: Measurement input configuration for " + cfg.name +
            " does not provide consistent sizes for " + prefix);
      };

    setup_fusion_config("position_m", cfg.position_m, num_position);
    setup_fusion_config("orientation_rad", cfg.orientation_rad, num_orientation);
    setup_fusion_config("linear_velocity_mps", cfg.linear_velocity_mps, num_linear_velocity);
    measurement_channels.push_back(channel);
  }

  // Match config to Input Configuration Data Structure
  uint8_t num_imu{};
  for (const auto& [name, parameter] : input_map) {
    tam::types::state::input channel{};
    tam::types::state::config::input& cfg = channel.config;
    cfg.name = name;

    if (!parameter.count("message.topic"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name + " does not provide a topic");
    if (!parameter.count("message.type"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name +
        " does not provide a topic type");

    cfg.measurement_sub.topic = parameter.at("message.topic").template get<std::string>();
    cfg.measurement_sub.type = parameter.at("message.type").template get<std::string>();
    cfg.measurement_sub.timeout_ms =
      parameter.count("message.timeout_ms") ? parameter.at("message.timeout_ms").template get<std::int32_t>()
                                            : 500;
    cfg.measurement_sub.active = true;

    if (!is_imu_type(cfg.measurement_sub.type))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name +
        " has an unimplemented topic type '" + cfg.measurement_sub.type +
        "', only 'sensor_msgs/Imu' (or 'sensor_msgs/msg/Imu') is supported");

    cfg.filter_coefficients = parameter.count("filter_coefficients")
                                ? parameter.at("filter_coefficients").template get<std::vector<double>>()
                                : std::vector<double>({1.0});
    cfg.backup = parameter.count("backup") ? parameter.at("backup").template get<bool>() : false;
    cfg.num = !cfg.backup ? num_imu++
                          : TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1;
    identifier_names.emplace_back(
      tam::types::state::measurements::identifier{tam::types::state::measurements::IMU, cfg.num},
      cfg.name);
    input_channels.push_back(channel);

    param_manager_raw->set_value(
      "imu_" + std::to_string(cfg.num + 1) + "_filter_coefficients", cfg.filter_coefficients);
  }
}

}  // namespace tam::core::state
